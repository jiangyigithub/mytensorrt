import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.nn.modules.batchnorm import _BatchNorm


def kaiming_init(module,
                 a=0,
                 mode='fan_out',
                 nonlinearity='relu',
                 bias=0,
                 distribution='normal'):
    assert distribution in ['uniform', 'normal']
    if hasattr(module, 'weight') and module.weight is not None:
        if distribution == 'uniform':
            nn.init.kaiming_uniform_(module.weight,
                                     a=a,
                                     mode=mode,
                                     nonlinearity=nonlinearity)
        else:
            nn.init.kaiming_normal_(module.weight,
                                    a=a,
                                    mode=mode,
                                    nonlinearity=nonlinearity)
    if hasattr(module, 'bias') and module.bias is not None:
        nn.init.constant_(module.bias, bias)


def constant_init(module, val, bias=0):
    if hasattr(module, 'weight') and module.weight is not None:
        nn.init.constant_(module.weight, val)
    if hasattr(module, 'bias') and module.bias is not None:
        nn.init.constant_(module.bias, bias)


class HSigmoidv2(nn.Module):
    """(add ref)"""
    def __init__(self, inplace=True):
        super(HSigmoidv2, self).__init__()
        self.inplace = inplace

    def forward(self, x):
        out = F.relu6(x + 3., inplace=self.inplace) / 6.
        return out


class AttnWeights(nn.Module):
    """Attention weights for the mixture of affine transformations
    https://arxiv.org/abs/1908.01259."""
    def __init__(self,
                 attn_mode,
                 num_features,
                 num_affine_trans,
                 num_groups=1,
                 use_rsd=True,
                 use_maxpool=False,
                 use_bn=True,
                 eps=1e-3):
        super(AttnWeights, self).__init__()

        if use_rsd:
            use_maxpool = False

        self.num_affine_trans = num_affine_trans
        self.use_rsd = use_rsd
        self.use_maxpool = use_maxpool
        self.eps = eps
        if not self.use_rsd:
            self.avgpool = nn.AdaptiveAvgPool2d(1)

        layers = []
        if attn_mode == 0:
            layers = [
                nn.Conv2d(num_features, num_affine_trans, 1, bias=not use_bn),
                nn.BatchNorm2d(num_affine_trans) if use_bn else nn.Identity(),
                HSigmoidv2()
            ]
        elif attn_mode == 1:
            if num_groups > 0:
                assert num_groups <= num_affine_trans
                layers = [
                    nn.Conv2d(num_features, num_affine_trans, 1, bias=False),
                    nn.GroupNorm(num_channels=num_affine_trans,
                                 num_groups=num_groups),
                    HSigmoidv2()
                ]
            else:
                layers = [
                    nn.Conv2d(num_features, num_affine_trans, 1, bias=False),
                    nn.BatchNorm2d(num_affine_trans)
                    if use_bn else nn.Identity(),
                    HSigmoidv2()
                ]
        else:
            raise NotImplementedError('Unknow attention weight type')

        self.attention = nn.Sequential(*layers)

        self.init_weights()

    def init_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                kaiming_init(m)
            elif isinstance(m, (_BatchNorm, nn.GroupNorm)):
                constant_init(m, 1)

    def forward(self, x):
        b, c, h, w = x.size()
        if self.use_rsd:
            var, mean = torch.var_mean(x, dim=(2, 3), keepdim=True)
            y = mean * (var + self.eps).rsqrt()

            # var = torch.var(x, dim=(2, 3), keepdim=True)
            # y *= (var + self.eps).rsqrt()
        else:
            y = self.avgpool(x)
            if self.use_maxpool:
                y += F.max_pool2d(x, (h, w), stride=(h, w)).view(b, c, 1, 1)
        return self.attention(y).view(b, self.num_affine_trans)


class AttnBatchNorm2d(nn.BatchNorm2d):
    """Attentive Normalization with BatchNorm2d backbone
    https://arxiv.org/abs/1908.01259."""

    _abbr_ = 'AttnBN2d'

    def __init__(self,
                 num_features,
                 num_affine_trans,
                 attn_mode=0,
                 eps=1e-5,
                 momentum=0.1,
                 track_running_stats=True,
                 use_rsd=True,
                 use_maxpool=False,
                 use_bn=True,
                 eps_var=1e-3):
        super(AttnBatchNorm2d,
              self).__init__(num_features,
                             affine=False,
                             eps=eps,
                             momentum=momentum,
                             track_running_stats=track_running_stats)

        self.num_affine_trans = num_affine_trans
        self.attn_mode = attn_mode
        self.use_rsd = use_rsd
        self.eps_var = eps_var

        self.weight_ = nn.Parameter(
            torch.Tensor(num_affine_trans, num_features))
        self.bias_ = nn.Parameter(torch.Tensor(num_affine_trans, num_features))

        self.attn_weights = AttnWeights(attn_mode,
                                        num_features,
                                        num_affine_trans,
                                        use_rsd=use_rsd,
                                        use_maxpool=use_maxpool,
                                        use_bn=use_bn,
                                        eps=eps_var)

        self.init_weights()

    def init_weights(self):
        nn.init.normal_(self.weight_, 1., 0.1)
        nn.init.normal_(self.bias_, 0., 0.1)

    def forward(self, x):
        output = super(AttnBatchNorm2d, self).forward(x)
        size = output.size()
        y = self.attn_weights(x)  # bxk

        weight = y @ self.weight_  # bxc
        bias = y @ self.bias_  # bxc
        weight = weight.unsqueeze(-1).unsqueeze(-1).expand(size)
        bias = bias.unsqueeze(-1).unsqueeze(-1).expand(size)

        return weight * output + bias
