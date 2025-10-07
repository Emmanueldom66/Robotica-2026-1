from Calculadora import Calculadora2params
from CalculadoraMultiple import CalculadoraMultParams

print("Operaciones aritmeticas básicas de dos paŕametros: ")

calc1 = Calculadora2params(2,5)
calc1.__str__()

print("Operaciones aritméticas de múltiples parámetros: ")

calc2 = CalculadoraMultParams(1,2,3,4,5,6,7)
calc2.__str__()