# Based on nuScenes dev-kit written
# by Holger Caesar, Varun Bankiti, and Alex Lang, 2019.

import json
from typing import Any

import numpy as np
from detection.constants import (DETECTION_COLORS, DETECTION_NAMES,
                                 PRETTY_DETECTION_NAMES, PRETTY_TP_METRICS,
                                 TP_METRICS, TP_METRICS_UNITS)
from detection.data_classes import (DetectionMetricData,
                                    DetectionMetricDataList, DetectionMetrics)
from matplotlib import pyplot as plt

Axis = Any


def setup_axis(xlabel: str = None,
               ylabel: str = None,
               xlim: int = None,
               ylim: int = None,
               title: str = None,
               min_precision: float = None,
               min_recall: float = None,
               ax: Axis = None,
               show_spines: str = 'none'):
    """Helper method that sets up the axis for a plot.

    :param xlabel: x label text.
    :param ylabel: y label text.
    :param xlim: Upper limit for x axis.
    :param ylim: Upper limit for y axis.
    :param title: Axis title.
    :param min_precision: Visualize minimum precision as horizontal line.
    :param min_recall: Visualize minimum recall as vertical line.
    :param ax: (optional) an existing axis to be modified.
    :param show_spines: Whether to show axes spines, set to 'none' by default.
    :return: The axes object.
    """
    if ax is None:
        ax = plt.subplot()

    ax.get_xaxis().tick_bottom()
    ax.tick_params(labelsize=16)
    ax.get_yaxis().tick_left()

    # Hide the selected axes spines.
    if show_spines in ['bottomleft', 'none']:
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

        if show_spines == 'none':
            ax.spines['bottom'].set_visible(False)
            ax.spines['left'].set_visible(False)
    elif show_spines in ['all']:
        pass
    else:
        raise NotImplementedError

    if title is not None:
        ax.set_title(title, size=24)
    if xlabel is not None:
        ax.set_xlabel(xlabel, size=16)
    if ylabel is not None:
        ax.set_ylabel(ylabel, size=16)
    if xlim is not None:
        ax.set_xlim(0, xlim)
    if ylim is not None:
        ax.set_ylim(0, ylim)
    if min_recall is not None:
        ax.axvline(x=min_recall, linestyle='--', color=(0, 0, 0, 0.3))
    if min_precision is not None:
        ax.axhline(y=min_precision, linestyle='--', color=(0, 0, 0, 0.3))

    return ax


def class_pr_curve(md_list: DetectionMetricDataList,
                   metrics: DetectionMetrics,
                   detection_name: str,
                   min_precision: float,
                   min_recall: float,
                   savepath: str = None,
                   ax: Axis = None) -> None:
    """Plot a precision recall curve for the specified class.

    :param md_list: DetectionMetricDataList instance.
    :param metrics: DetectionMetrics instance.
    :param detection_name: The detection class.
    :param min_precision:
    :param min_recall: Minimum recall value.
    :param savepath: If given, saves the the rendering
    here instead of displaying.
    :param ax: Axes onto which to render.
    """
    # Prepare axis.
    if ax is None:
        ax = setup_axis(title=PRETTY_DETECTION_NAMES[detection_name],
                        xlabel='Recall',
                        ylabel='Precision',
                        xlim=1,
                        ylim=1,
                        min_precision=min_precision,
                        min_recall=min_recall)

    # Get recall vs precision values of given class
    # for each distance threshold.
    data = md_list.get_class_data(detection_name)

    # Plot the recall vs. precision curve for each distance threshold.
    for md, dist_th in data:
        md: DetectionMetricData
        ap = metrics.get_label_ap(detection_name, dist_th)
        ax.plot(md.recall,
                md.precision,
                label='Dist. : {}, AP: {:.1f}'.format(dist_th, ap * 100))

    ax.legend(loc='best')
    if savepath is not None:
        plt.savefig(savepath)
        plt.close()


def class_tp_curve(md_list: DetectionMetricDataList,
                   metrics: DetectionMetrics,
                   detection_name: str,
                   min_recall: float,
                   dist_th_tp: float,
                   savepath: str = None,
                   ax: Axis = None) -> None:
    """Plot the true positive curve for the specified class.

    :param md_list: DetectionMetricDataList instance.
    :param metrics: DetectionMetrics instance.
    :param detection_name:
    :param min_recall: Minimum recall value.
    :param dist_th_tp: The distance threshold used to determine matches.
    :param savepath: If given, saves the the rendering here
    instead of displaying.
    :param ax: Axes onto which to render.
    """
    # Get metric data for given detection class with tp distance threshold.
    md = md_list[(detection_name, dist_th_tp)]
    min_recall_ind = round(100 * min_recall)
    if min_recall_ind <= md.max_recall_ind:
        # For traffic_cone and barrier only a subset
        # of the metrics are plotted.
        rel_metrics = [
            m for m in TP_METRICS
            if not np.isnan(metrics.get_label_tp(detection_name, m))
        ]
        ylimit = max([
            max(getattr(md, metric)[min_recall_ind:md.max_recall_ind + 1])
            for metric in rel_metrics
        ]) * 1.1
    else:
        ylimit = 1.0

    # Prepare axis.
    if ax is None:
        ax = setup_axis(title=PRETTY_DETECTION_NAMES[detection_name],
                        xlabel='Recall',
                        ylabel='Error',
                        xlim=1,
                        min_recall=min_recall)
    ax.set_ylim(0, ylimit)

    # Plot the recall vs. error curve for each tp metric.
    for metric in TP_METRICS:
        tp = metrics.get_label_tp(detection_name, metric)

        # Plot only if we have valid data.
        if tp is not np.nan and min_recall_ind <= md.max_recall_ind:
            recall, error = md.recall[:md.max_recall_ind + 1], getattr(
                md, metric)[:md.max_recall_ind + 1]
        else:
            recall, error = [], []

        # Change legend based on tp value
        if tp is np.nan:
            label = '{}: n/a'.format(PRETTY_TP_METRICS[metric])
        elif min_recall_ind > md.max_recall_ind:
            label = '{}: nan'.format(PRETTY_TP_METRICS[metric])
        else:
            label = '{}: {:.2f} ({})'.format(PRETTY_TP_METRICS[metric], tp,
                                             TP_METRICS_UNITS[metric])
        ax.plot(recall, error, label=label)
    ax.axvline(x=md.max_recall, linestyle='-.', color=(0, 0, 0, 0.3))
    ax.legend(loc='best')

    if savepath is not None:
        plt.savefig(savepath)
        plt.close()


def dist_pr_curve(md_list: DetectionMetricDataList,
                  metrics: DetectionMetrics,
                  dist_th: float,
                  min_precision: float,
                  min_recall: float,
                  savepath: str = None) -> None:
    """Plot the PR curves for different distance thresholds.

    :param md_list: DetectionMetricDataList instance.
    :param metrics: DetectionMetrics instance.
    :param dist_th: Distance threshold for matching.
    :param min_precision: Minimum precision value.
    :param min_recall: Minimum recall value.
    :param savepath: If given, saves the the rendering here
    instead of displaying.
    """
    # Prepare axis.
    fig, (ax, lax) = plt.subplots(ncols=2,
                                  gridspec_kw={'width_ratios': [4, 1]},
                                  figsize=(7.5, 5))
    ax = setup_axis(xlabel='Recall',
                    ylabel='Precision',
                    xlim=1,
                    ylim=1,
                    min_precision=min_precision,
                    min_recall=min_recall,
                    ax=ax)

    # Plot the recall vs. precision curve for each detection class.
    data = md_list.get_dist_data(dist_th)
    for md, detection_name in data:
        md = md_list[(detection_name, dist_th)]
        ap = metrics.get_label_ap(detection_name, dist_th)
        ax.plot(md.recall,
                md.precision,
                label='{}: {:.1f}%'.format(
                    PRETTY_DETECTION_NAMES[detection_name], ap * 100),
                color=DETECTION_COLORS[detection_name])
    hx, lx = ax.get_legend_handles_labels()
    lax.legend(hx, lx, borderaxespad=0)
    lax.axis('off')
    plt.tight_layout()
    if savepath is not None:
        plt.savefig(savepath)
        plt.close()


def summary_plot(md_list: DetectionMetricDataList,
                 metrics: DetectionMetrics,
                 min_precision: float,
                 min_recall: float,
                 dist_th_tp: float,
                 savepath: str = None) -> None:
    """Creates a summary plot with PR and TP curves for each class.

    :param md_list: DetectionMetricDataList instance.
    :param metrics: DetectionMetrics instance.
    :param min_precision: Minimum precision value.
    :param min_recall: Minimum recall value.
    :param dist_th_tp: The distance threshold used to determine matches.
    :param savepath: If given, saves the the rendering here
    instead of displaying.
    """
    n_classes = len(DETECTION_NAMES)
    _, axes = plt.subplots(nrows=n_classes,
                           ncols=2,
                           figsize=(15, 5 * n_classes))
    for ind, detection_name in enumerate(DETECTION_NAMES):
        title1, title2 = ('Recall vs Precision',
                          'Recall vs Error') if ind == 0 else (None, None)

        ax1 = setup_axis(xlim=1,
                         ylim=1,
                         title=title1,
                         min_precision=min_precision,
                         min_recall=min_recall,
                         ax=axes[ind, 0])
        ax1.set_ylabel('{} \n \n Precision'.format(
            PRETTY_DETECTION_NAMES[detection_name]),
                       size=20)

        ax2 = setup_axis(xlim=1,
                         title=title2,
                         min_recall=min_recall,
                         ax=axes[ind, 1])
        if ind == n_classes - 1:
            ax1.set_xlabel('Recall', size=20)
            ax2.set_xlabel('Recall', size=20)

        class_pr_curve(md_list,
                       metrics,
                       detection_name,
                       min_precision,
                       min_recall,
                       ax=ax1)
        class_tp_curve(md_list,
                       metrics,
                       detection_name,
                       min_recall,
                       dist_th_tp=dist_th_tp,
                       ax=ax2)

    plt.tight_layout()

    if savepath is not None:
        plt.savefig(savepath)
        plt.close()


def detailed_results_table_tex(metrics_path: str, output_path: str) -> None:
    """Renders a detailed results table in tex.

    :param metrics_path: path to a serialized DetectionMetrics file.
    :param output_path: path to the output file.
    """
    with open(metrics_path, 'r') as f:
        metrics = json.load(f)

    tex = ''
    tex += '\\begin{table}[]\n'
    tex += '\\small\n'
    tex += '\\begin{tabular}{| c | c | c | c | c | c | c |} \\hline\n'
    tex += '\\textbf{Class} &' \
           '\\textbf{AP} & \\textbf{ATE} & \\textbf{ASE} & \\textbf{AOE} & ' \
           '\\textbf{AVE}   & ' \
           '\\textbf{AAE}   \\\\ \\hline ' \
           '\\hline\n'
    for name in DETECTION_NAMES:
        ap = np.mean(metrics['label_aps'][name].values()) * 100
        ate = metrics['label_tp_errors'][name]['trans_err']
        ase = metrics['label_tp_errors'][name]['scale_err']
        aoe = metrics['label_tp_errors'][name]['orient_err']
        ave = metrics['label_tp_errors'][name]['vel_err']
        aae = metrics['label_tp_errors'][name]['attr_err']
        tex_name = PRETTY_DETECTION_NAMES[name]
        if name == 'traffic_cone':
            tex += '{} & {:.1f} & {:.2f} & {:.2f} & N/A &' \
                    'N/A  &   N/A  \\\\ \\hline\n'.format(
                        tex_name, ap, ate, ase)
        elif name == 'barrier':
            tex += '{}  &   {:.1f}  &   {:.2f}  &   {:.2f}  &   {:.2f}  &' \
                    'N/A  &   N/A  \\\\ \\hline\n'.format(
                        tex_name, ap, ate, ase, aoe)
        else:
            tex += '{}  &   {:.1f}  &   {:.2f}  &   {:.2f}  &   {:.2f}  &' \
                '{:.2f}  &   {:.2f}  \\\\ ' \
                '\\hline\n'.format(tex_name, ap, ate, ase, aoe, ave, aae)

    map_ = metrics['mean_ap']
    mate = metrics['tp_errors']['trans_err']
    mase = metrics['tp_errors']['scale_err']
    maoe = metrics['tp_errors']['orient_err']
    mave = metrics['tp_errors']['vel_err']
    maae = metrics['tp_errors']['attr_err']
    tex += '\\hline {} &   {:.1f}  &   {:.2f}  &   {:.2f}  &   {:.2f}  &'\
        '{:.2f}  &   {:.2f}  \\\\ ' \
           '\\hline\n'.format('\\textbf{Mean}', map_,
                              mate, mase, maoe, mave, maae)

    tex += '\\end{tabular}\n'

    # All one line
    tex += '\\caption{Detailed detection performance on the val set. \n'
    tex += 'AP: average precision averaged over distance thresholds (%), \n'
    tex += 'ATE: average translation error (${}$), \n'.format(
        TP_METRICS_UNITS['trans_err'])
    tex += 'ASE: average scale error (${}$), \n'.format(
        TP_METRICS_UNITS['scale_err'])
    tex += 'AOE: average orientation error (${}$), \n'.format(
        TP_METRICS_UNITS['orient_err'])
    tex += 'AVE: average velocity error (${}$), \n'.format(
        TP_METRICS_UNITS['vel_err'])
    tex += 'AAE: average attribute error (${}$). \n'.format(
        TP_METRICS_UNITS['attr_err'])
    tex += 'nuScenes Detection Score (NDS) = {:.1f} \n'.format(
        metrics['nd_score'] * 100)
    tex += '}\n'

    tex += '\\end{table}\n'

    with open(output_path, 'w') as f:
        f.write(tex)
