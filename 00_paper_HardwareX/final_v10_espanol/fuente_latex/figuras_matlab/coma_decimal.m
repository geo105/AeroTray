function coma_decimal(ax)
%COMA_DECIMAL Escribe los numeros de los ejes con coma decimal y signo menos tipografico.
%   En ejes logaritmicos usa 10^{n} en las decadas. Llamar despues de fijar limites y marcas.
menos = char(8722);                          % signo menos tipografico
if exist('OCTAVE_VERSION', 'builtin'), menos = '-'; end   % Octave no admite no-ASCII en las marcas
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
            s = sprintf('%.6g', round(v*1e6)/1e6);
            etiquetas{k} = strrep(strrep(s, '.', ','), '-', menos);
        end
    end
    set(ax, [e 'Tick'], marcas, [e 'TickLabel'], etiquetas);
    try, set(ax, [e 'TickMode'], 'manual', [e 'TickLabelMode'], 'manual'); catch, end
end
end
