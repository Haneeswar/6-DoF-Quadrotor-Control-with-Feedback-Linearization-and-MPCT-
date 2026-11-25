function styleFigure(fig)
%STYLEFIGURE  Common look for all result figures: bold 14 pt axes, grids, 2 pt lines.

    axs = findall(fig, 'Type', 'axes');
    for i = 1:numel(axs)
        ax = axs(i);
        if strcmp(get(ax, 'Tag'), 'legend')
            continue;
        end
        set(ax, 'FontSize', 14, 'FontWeight', 'bold');
        set(ax, 'XGrid', 'on', 'YGrid', 'on', 'ZGrid', 'on', ...
                'XMinorGrid', 'on', 'YMinorGrid', 'on', 'ZMinorGrid', 'on');
        set(findall(ax, 'Type', 'line'), 'LineWidth', 2);
    end
    set(findall(fig, 'Type', 'legend'), 'FontSize', 12);
end
