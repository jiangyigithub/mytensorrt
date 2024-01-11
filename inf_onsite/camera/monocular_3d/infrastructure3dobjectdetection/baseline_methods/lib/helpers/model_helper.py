from lib.models.monocon import MonoCon
from lib.models.monodle import MonoDLE


def build_model(cfg):
    if cfg['type'] == 'monocon':
        return MonoCon(backbone=cfg['backbone'],
                       neck=cfg['neck'],
                       num_class=cfg['num_class'])
    elif cfg['type'] == 'monodle':
        return MonoDLE(backbone=cfg['backbone'],
                       neck=cfg['neck'],
                       num_class=cfg['num_class'])
    else:
        raise NotImplementedError('%s model is not supported' % cfg['type'])

def build_model_deploy():
    return MonoCon(backbone='dla34',
                    neck='DLAUp',
                    num_class=10)
