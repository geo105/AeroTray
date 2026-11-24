%GENERAR_TODAS Regenera todas las figuras del paper en ../figures/ con el mismo estilo.
%   1) Desde scripts_modelo/:  python exportar_datos_matlab.py  y  python exportar_ensayos_matlab.py
%      (calculan y guardan datos/*.csv; ya estan generados en esta version)
%   2) En MATLAB, desde esta carpeta:  generar_todas
%   Para cambiar fuente, tamanos o colores de TODAS las figuras, edita estilo_paper.m.
figuras = {'fig_base', 'fig_identificacion', 'fig_rampa', 'fig_bateria', 'fig_estabilidad_dron', ...
           'fig_lazo_bola', 'fig_transmisibilidad', 'fig_banco', 'fig_vuelos', ...
           'fig_bp_resultados', 'fig_ensayos_23nov', 'fig_bp23_despegue'};
for k = 1:numel(figuras)
    try
        feval(figuras{k});
    catch err
        fprintf(2, 'Error en %s: %s\n', figuras{k}, err.message);
    end
end
