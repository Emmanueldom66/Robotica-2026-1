class Calculadora2params:

  def __init__(self,a,b):
    self.a = a
    self.b = b

  def suma(self):
    suma2 = self.a + self.b
    return suma2
  
  def multiplicacion(self):
    mult = self.a * self.b
    return mult

  def __str__(self):
    print(f'''
          suma: {self.suma()}
          multiplicación: {self.multiplicacion()}''')