import argparse
import datetime
import os

import yaml
from lib.helpers.dataloader_helper import build_dataloader
from lib.helpers.model_helper import build_model
from lib.helpers.optimizer_helper import build_optimizer
from lib.helpers.scheduler_helper import build_lr_scheduler
from lib.helpers.tester_helper import Tester
from lib.helpers.trainer_helper import Trainer
from lib.helpers.utils_helper import create_logger, set_random_seed

parser = argparse.ArgumentParser(
    description='End-to-End Monocular 3D Object Detection')
parser.add_argument('--config',
                    dest='config',
                    help='settings of detection in yaml format')
parser.add_argument('-e',
                    '--evaluate_only',
                    action='store_true',
                    default=False,
                    help='evaluation only')

args = parser.parse_args()


def main():
    assert (os.path.exists(args.config))
    cfg = yaml.load(open(args.config, 'r'), Loader=yaml.Loader)
    set_random_seed(cfg.get('random_seed', 444))
    log_file = 'train.log.%s' % datetime.datetime.now().strftime(
        '%Y%m%d_%H%M%S')
    logger = create_logger(log_file)

    # build dataloader
    logger.info('###################  Build Dataloader  ##################')
    train_loader, test_loader = build_dataloader(cfg['dataset'], workers=1)
    logger.info(f'train_loader: {len(train_loader.dataset)}')
    logger.info(f'train_loader: {len(train_loader)}')

    # build model
    model = build_model(cfg['model'])

    if args.evaluate_only:
        logger.info('###################  Evaluation Only  ##################')
        tester = Tester(cfg=cfg['tester'],
                        model=model,
                        dataloader=test_loader,
                        logger=logger)
        tester.test()
        return

    #  build optimizer
    optimizer = build_optimizer(cfg['optimizer'], model)

    # build lr scheduler
    lr_scheduler, warmup_lr_scheduler = build_lr_scheduler(cfg['lr_scheduler'],
                                                           optimizer,
                                                           last_epoch=-1)

    logger.info('###################  Training  ##################')
    logger.info('Batch Size: %d' % (cfg['dataset']['batch_size']))
    logger.info('Learning Rate: %f' % (cfg['optimizer']['lr']))
    trainer = Trainer(cfg=cfg['trainer'],
                      model=model,
                      optimizer=optimizer,
                      train_loader=train_loader,
                      lr_scheduler=lr_scheduler,
                      warmup_lr_scheduler=warmup_lr_scheduler,
                      logger=logger)
    trainer.train()

    logger.info('###################  Evaluation  ##################')
    tester = Tester(cfg=cfg['tester'],
                    model=model,
                    dataloader=test_loader,
                    logger=logger)
    tester.test()


if __name__ == '__main__':
    main()
