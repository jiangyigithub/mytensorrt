import torch


def laplacian_aleatoric_uncertainty_loss(input,
                                         target,
                                         log_variance,
                                         reduction='mean'):
    assert reduction in ['mean', 'sum']
    loss = 1.4142 * torch.exp(-log_variance) * torch.abs(input -
                                                         target) + log_variance
    return loss.mean() if reduction == 'mean' else loss.sum()


def gaussian_aleatoric_uncertainty_loss(input,
                                        target,
                                        log_variance,
                                        reduction='mean'):
    assert reduction in ['mean', 'sum']
    loss = 0.5 * torch.exp(-log_variance) * torch.abs(
        input - target)**2 + 0.5 * log_variance
    return loss.mean() if reduction == 'mean' else loss.sum()
