import numpy as np
from lib.datasets.dair.dair_dataset import DAIR_Dataset_MonoCon
from torch.utils.data import DataLoader


# init datasets and dataloaders
def my_worker_init_fn(worker_id):
    np.random.seed(np.random.get_state()[1][0] + worker_id)


def build_dataloader(cfg, workers=1):
    if cfg['type'] == 'DAIR':
        train_set = DAIR_Dataset_MonoCon(split='train', cfg=cfg)
        test_set = DAIR_Dataset_MonoCon(split='val', cfg=cfg)
    else:
        raise NotImplementedError('%s dataset is not supported' % cfg['type'])

    # prepare dataloader
    train_loader = DataLoader(dataset=train_set,
                              batch_size=cfg['batch_size'],
                              num_workers=workers,
                              worker_init_fn=my_worker_init_fn,
                              shuffle=True,
                              pin_memory=False,
                              drop_last=True)
    test_loader = DataLoader(dataset=test_set,
                             batch_size=cfg['batch_size'],
                             num_workers=workers,
                             worker_init_fn=my_worker_init_fn,
                             shuffle=False,
                             pin_memory=False,
                             drop_last=False)

    return train_loader, test_loader
