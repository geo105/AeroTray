import re as _re, matplotlib.ticker as _tk, matplotlib.text as _mt
def es(fig):
    """Coma decimal en textos y ejes lineales."""
    for t in fig.findobj(_mt.Text):
        tx=t.get_text()
        if tx: t.set_text(_re.sub(r"(\d)\.(\d)",r"\1,\2",tx))
    for a in fig.axes:
        for ax_ in (a.xaxis,a.yaxis):
            if ax_.get_scale()=="linear" and isinstance(ax_.get_major_formatter(),_tk.ScalarFormatter):
                ax_.set_major_formatter(_tk.FuncFormatter(lambda v,p: ("%g"%round(v,6)).replace(".",",").replace("-","−")))
