# ReadMe

This is a fork of [SymPyBotics](https://github.com/cdsousa/SymPyBotics.git). I add some features to generate MATLAB code from SymPy expressions. Some bugs are also fixed.

A demo to identify franka robot parameters is provided in `scripts/identify_franka.py`.

## package

```shell
pip install sympy==1.8
pip install setuptools
pip install numpy
```

## About SymPyBotics

+ Modify `sympybotics/symcode/subexprs.py`, Line 142
```python
if sympy.utilities.iterables.iterable(expr):
    return expr
```
+ Add `gen_matlab_func` to `sympybotics/symcode/generation.py`
```python
def _mcode(expr):
    code = sympy.printing.octave.OctaveCodePrinter().doprint(expr)
    if options.get('unroll_square', False):
        code = re.sub(r'(\w+)\s*\.\^\s*2', r'(\1).*(\1)', code)
    return code


def zero_to_one_index(match):
    var = match.group(1)
    idx = match.group(2)
    try:
        new_idx = str(int(idx) + 1)
    except ValueError:
        new_idx = f'{idx} + 1'
    return f'{var}({new_idx})'

def gen_matlab_func(code, out_parms, func_parms, func_name='func'):
    indent = 4 * ' '

    out_str = ', '.join(out_parms) if len(out_parms) > 1 else out_parms[0]
    in_str = ', '.join(func_parms)

    mcode = f'function [{out_str}] = {func_name}({in_str})\n\n'

    mcode += code_to_string(code, out_parms, _mcode, indent, '', ';')
    
    mcode += '\nend\n'

    mcode = mcode.replace('\n\n', '\n% \n')

    # a[0] -> a(1)
    mcode = re.sub(r'(\w)\[([^\[\]]+?)\]', zero_to_one_index, mcode)

    return mcode
```
We get
```matlab
function [tau_out] = tau(parms, q, dq, ddq)
% 
    x0 = -ddq(1);
    x1 = sin(q(2));
    x2 = dq(1);
    x3 = x1.*x2;
    x4 = -x3;
    x5 = cos(q(2));
    x6 = -x0;
    x7 = dq(2).*x4 + x5.*x6;
    x8 = x2.*x5;
    x9 = dq(2).*x8 + x1.*x6;
    x10 = dq(2).*parms(17) + parms(15).*x8 + parms(18).*x3;
    x11 = 9.81*x1;
    x12 = dq(2).*parms(16) + parms(14).*x8 + parms(17).*x3;
    x13 = dq(2).*parms(14) + parms(13).*x8 + parms(15).*x3;
    x14 = 9.81*x5;
% 
    tau_out(1) = parms(11).*sign(dq(1)) + parms(12) - parms(4).*x0 + x1.*(ddq(2).*parms(17) - dq(2).*x13 + parms(15).*x7 + parms(18).*x9 - parms(20).*x14 + x12.*x8) + x5.*(ddq(2).*parms(14) + dq(2).*x10 + parms(13).*x7 + parms(15).*x9 + parms(20).*x11 + x12.*x4);
    tau_out(2) = ddq(2).*parms(16) + parms(14).*x7 + parms(17).*x9 - parms(19).*x11 + parms(21).*x14 + parms(23).*sign(dq(2)) + parms(24) - x10.*x8 + x13.*x3;
% 
end
```
+ Line 57-59, `dynamics.py`
```python
self.Pb = sympy.Matrix(numpy.array(Pb).tolist()).applyfunc(lambda x: x.nsimplify())
self.Pd = sympy.Matrix(numpy.array(Pd).tolist()).applyfunc(lambda x: x.nsimplify())
self.Kd = sympy.Matrix(numpy.array(Kd).tolist()).applyfunc(lambda x: x.nsimplify())
```
+ `robotmodel.py`, `def calc_base_parms`, modify:
```python
global sin, cos, sign
sin = numpy.sin
cos = numpy.cos
sign = numpy.sign

l = locals()
exec_(func_def_regressor, globals(), l)
```
as 
```python
import math
global sin, cos, sign
sin = math.sin
cos = math.cos
sign = numpy.sign

l = locals()
# exec_(func_def_regressor, globals(), l)
exec_(func_def_regressor, {'math': math, 'sin': sin, 'cos': cos, 'sign': sign}, l)
```