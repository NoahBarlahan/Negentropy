import numpy as np
import pytest
from config import defaults
from model import evaluate
from plots import view3d,orthographic
from units import format_length,length_rows,length_figure


def test_lengths_tables_and_csv_headers_keep_other_quantities():
    assert format_length(25.4,'in')=='1.000 in'
    assert format_length(-50.8,'in')=='-2.000 in'
    assert length_rows([{'thigh (mm)':76.2,'torque (N·m)':1.2,'status':'ok'}],'in')==[
        {'thigh (in)':pytest.approx(3),'torque (N·m)':1.2,'status':'ok'}]


@pytest.mark.parametrize('factory',[view3d,orthographic])
def test_figures_convert_coordinates_labels_and_preserve_proportions(factory):
    e=evaluate(defaults())
    mm=factory(e);inches=length_figure(factory(e),'in')
    for original,converted in zip(mm.data,inches.data):
        for axis in 'xyz':
            before=getattr(original,axis,None)
            if before is not None:
                np.testing.assert_allclose(np.asarray(getattr(converted,axis),dtype=float)*25.4,np.asarray(before,dtype=float),equal_nan=True)
    if factory==view3d:
        assert inches.layout.scene.xaxis.title.text=='Forward x (in)'
        np.testing.assert_allclose(np.array(inches.layout.scene.xaxis.range)*25.4,mm.layout.scene.xaxis.range)
        assert inches.layout.scene.aspectratio==mm.layout.scene.aspectratio
        assert any('2.559 in' in str(trace.text) for trace in inches.data)
    else:
        assert inches.layout.xaxis.title.text=='x (in)'
