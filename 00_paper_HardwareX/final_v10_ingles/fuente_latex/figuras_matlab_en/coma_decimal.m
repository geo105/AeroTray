function coma_decimal(ax)
%COMA_DECIMAL Format axis tick numbers with English decimal points and typographic minus sign.
%   Uses 10^{n} on log decades. Call after setting limits and ticks.
if exist('OCTAVE_VERSION', 'builtin')
    menos = '-';                             % Octave does not accept non-ASCII in tick labels
else
    menos = char(8722);                      % typographic minus sign
end
for eje = {'X', 'Y'}
    e = eje{1};
    marcas = get(ax, [e 'Tick']);
    if isempty(marcas), continue; end
    esc = get(ax, [e 'Scale']);
    etiquetas = cell(size(marcas));
    for k = 1:numel(marcas)
        v = marcas(k);
        if strcmp(esc, 'log') && abs(log10(v) - round(log10(v))) < 1e-9
            etiquetas{k} = sprintf('10^{%d}', round(log10(v)));
        else
            rv = round(v*1e6)/1e6;
            if rv == 0, rv = 0; end
            s = sprintf('%.6g', rv);
            etiquetas{k} = strrep(s, '-', menos);
        end
    end
    set(ax, [e 'Tick'], marcas, [e 'TickLabel'], etiquetas);
    try, set(ax, [e 'TickMode'], 'manual', [e 'TickLabelMode'], 'manual'); catch, end
end
end
