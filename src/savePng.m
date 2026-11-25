function savePng(fig, file)
%SAVEPNG  Write a figure as a PNG file with the renderer that suits the environment.
    if exist('OCTAVE_VERSION', 'builtin')
        print(fig, file, '-dpngcairo', '-r110');
    else
        print(fig, file, '-dpng', '-r110');
    end
end
