import math
import os
import time

import numpy as np
import torch
import tqdm
from lib.helpers.save_helper import (get_checkpoint_state, load_checkpoint,
                                     save_checkpoint)
from lib.losses.centernet_loss import compute_centernet3d_loss
from progress.bar import Bar
from tensorboardX import SummaryWriter
from torchvision.utils import make_grid


def log10(x):
    return torch.log(x) / math.log(10)


class AverageMeter(object):
    """Computes and stores the average and current value."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.val = 0
        self.avg = 0
        self.sum = 0
        self.count = 0

    def update(self, val, n=1):
        self.val = val
        self.sum += val * n
        self.count += n
        if self.count > 0:
            self.avg = self.sum / self.count


class Trainer(object):
    def __init__(self, cfg, model, optimizer, train_loader, lr_scheduler,
                 warmup_lr_scheduler, logger):
        self.cfg = cfg
        self.model = model
        self.optimizer = optimizer
        self.train_loader = train_loader
        # self.test_loader = test_loader
        self.lr_scheduler = lr_scheduler
        self.warmup_lr_scheduler = warmup_lr_scheduler
        self.logger = logger
        self.epoch = 0
        self.step = 0
        self.device = torch.device(
            'cuda:0' if torch.cuda.is_available() else 'cpu')
        self.tensorboard_interval = 20
        # loading pretrain/resume model
        if cfg.get('pretrain_model'):
            assert os.path.exists(cfg['pretrain_model'])
            load_checkpoint(model=self.model,
                            optimizer=None,
                            filename=cfg['pretrain_model'],
                            map_location=self.device,
                            logger=self.logger)

        if cfg.get('resume_model', None):
            assert os.path.exists(cfg['resume_model'])
            self.epoch = load_checkpoint(model=self.model.to(self.device),
                                         optimizer=self.optimizer,
                                         filename=cfg['resume_model'],
                                         map_location=self.device,
                                         logger=self.logger)
            self.lr_scheduler.last_epoch = self.epoch - 1

        self.gpu_ids = list(map(int, cfg['gpu_ids'].split(',')))
        self.model = torch.nn.DataParallel(model).cuda()
        logdir = f'./runs/logs/{time.time()}'
        os.makedirs(logdir, exist_ok=True)
        self.writer = SummaryWriter(logdir)

    def train(self):
        start_epoch = self.epoch

        progress_bar = tqdm.tqdm(range(start_epoch, self.cfg['max_epoch']),
                                 dynamic_ncols=True,
                                 leave=True,
                                 desc='epochs')
        for epoch in range(start_epoch, self.cfg['max_epoch']):
            # reset random seed
            # ref: https://github.com/pytorch/pytorch/issues/5059
            np.random.seed(np.random.get_state()[1][0] + epoch)
            # train one epoch
            self.train_one_epoch()
            self.epoch += 1

            # update learning rate
            if self.warmup_lr_scheduler is not None and epoch < 5:
                self.warmup_lr_scheduler.step()
            else:
                self.lr_scheduler.step()

            if (self.epoch == 1):
                os.makedirs(self.cfg['save_dir'], exist_ok=True)
                ckpt_name = os.path.join(self.cfg['save_dir'],
                                         'checkpoint_epoch_%d' % self.epoch)
                save_checkpoint(
                    get_checkpoint_state(self.model, self.optimizer,
                                         self.epoch), ckpt_name)

            # save trained model
            if (self.epoch % self.cfg['save_frequency']) == 0:
                os.makedirs(self.cfg['save_dir'], exist_ok=True)
                ckpt_name = os.path.join(self.cfg['save_dir'],
                                         'checkpoint_epoch_%d' % self.epoch)
                save_checkpoint(
                    get_checkpoint_state(self.model, self.optimizer,
                                         self.epoch), ckpt_name)

                # self.inference()

            progress_bar.update()

        return None

    def train_one_epoch(self):
        self.model.train()
        self.stats = {}  # reset stats dict
        self.stats['train'] = {}  # reset stats dict
        loss_stats = [
            'seg', 'offset2d', 'size2d', 'offset3d', 'depth', 'size3d',
            'heading', 'center2kpt_offset', 'kpt_heatmap', 'kpt_heatmap_offset'
        ]
        _, batch_time = AverageMeter(), AverageMeter()
        avg_loss_stats = {loss: AverageMeter() for loss in loss_stats}
        num_iters = len(self.train_loader)
        bar = Bar('{}/{}'.format('3D', self.cfg['save_dir']), max=num_iters)
        end = time.time()
        print(f'in trainer {len(self.train_loader)}')
        for batch_idx, (inputs, targets,
                        infos) in enumerate(self.train_loader):
            # print(targets)
            inputs = inputs.to(self.device)
            for key in targets.keys():
                targets[key] = targets[key].to(self.device)

            # train one batch
            self.optimizer.zero_grad()
            outputs = self.model(inputs)
            total_loss, stats_batch = compute_centernet3d_loss(
                outputs, targets)
            total_loss.backward()
            self.optimizer.step()

            if batch_idx % self.tensorboard_interval == 0:
                for key in stats_batch:
                    self.writer.add_scalar(key, stats_batch[key], self.step)
                self.writer.add_scalar('total_loss', total_loss, self.step)

            batch_time.update(time.time() - end)
            end = time.time()
            Bar.suffix = \
                '{phase}: [{0}][{1}/{2}]|Tot: {total:} |ETA: {eta:} '.format(
                    self.epoch,
                    batch_idx,
                    num_iters,
                    phase='train',
                    total=bar.elapsed_td,
                    eta=bar.eta_td)
            self.step += 1
            if self.step % (num_iters // 10) == 0:
                self.writer.add_image('input', make_grid(inputs, nrow=4),
                                      self.step)
            for loss in avg_loss_stats:
                avg_loss_stats[loss].update(stats_batch[loss], inputs.shape[0])
                Bar.suffix = Bar.suffix + '|{} {:.4f} '.format(
                    loss, avg_loss_stats[loss].avg)

            bar.next()
        bar.finish()
