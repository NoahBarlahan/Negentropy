"""Presentation conversions; configurations remain in mm and the core in SI."""
import re
import numpy as np


def mm_per_unit(unit):
    return 25.4 if unit == "in" else 1.0


def length_label(label, unit):
    return label.replace("(mm)", f"({unit})")


def format_length(mm, unit):
    return f"{mm/mm_per_unit(unit):.{3 if unit == 'in' else 1}f} {unit}"


def length_rows(rows, unit):
    return [{length_label(k, unit): (v/mm_per_unit(unit) if "(mm)" in k and isinstance(v, (int,float,np.number)) else v)
             for k,v in row.items()} for row in rows]


def length_figure(fig, unit):
    """Convert all spatial coordinates uniformly, preserving physical proportions."""
    if unit == "mm":
        return fig
    factor=mm_per_unit(unit)
    def text(value):
        if not isinstance(value,str):
            return value
        value=re.sub(r"(-?\d+(?:\.\d+)?) mm",lambda m:format_length(float(m[1]),unit),value)
        return value.replace("(mm)","(in)").replace(" mm<", " in<").replace(":.1f}",":.3f}")
    for trace in fig.data:
        for axis in "xyz":
            values=getattr(trace,axis,None)
            if values is not None:
                setattr(trace,axis,np.asarray(values,dtype=float)/factor)
        if trace.text is not None:
            trace.text=text(trace.text) if isinstance(trace.text,str) else [text(t) for t in trace.text]
        if trace.hovertemplate is not None:
            trace.hovertemplate=text(trace.hovertemplate)
    scene=fig.layout.scene
    for axis in "xyz":
        ax=getattr(scene,axis+"axis")
        if ax.range:
            ax.range=[v/factor for v in ax.range]
        if ax.title.text:
            ax.title.text=text(ax.title.text)
    for ax in [*fig.select_xaxes(),*fig.select_yaxes()]:
        if ax.title.text:
            ax.title.text=text(ax.title.text)
        if ax.range:
            ax.range=[v/factor for v in ax.range]
    fig.layout.uirevision=f"robot-camera-{unit}"
    return fig
