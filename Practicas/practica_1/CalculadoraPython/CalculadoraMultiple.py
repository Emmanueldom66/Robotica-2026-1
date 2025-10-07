class CalculadoraMultParams:

  def __init__(self, * args):
    self.args = args

  def suma(self):
    return sum(self.args)
  
  def multiplicacion(self):
    mult = 1
    for num in self.args:
      mult *= num
    return mult

  def __str__(self):
    print(f'''
          suma: {self.suma()}
          multiplicación: {self.multiplicacion()}''')