class Decision:      

    def check_front(self):
        try: 
            #Si cumple las condiciones no hay nada delante osea que se devuelve False
            if(self.distances["frontal"] <=5   and self.distances["frontal"] >= 3 ):
                return False
            else:
                #Sino si que hay algo y devulve True
                return True

                
        except:
            # Si no tiene algo o si no encuentra el parametro dentro de el array no ha encontrado nada delante, osea False 
            return True

    def check_back(self):
        try:
            #Si hay algo detras pero esta lejos lo trata como que aun no pasa nada
            if(self.distances["back"] <=5   and self.distances["back"] >= 3 ):
                return False
            
            #Si esta demasiado cerca ya es un problema
            else:
                return True
            
        #Si pasa por el except significa que no hay nada detras del coche   
        except:
            return False
        
    def check_front_rigth(self):
        try:
            #Si hay algo delante a la derecha devulve True
            if(self.distances["frontal_right"] != 0):
                return True
            
            #Sino False
            else:
                return False
            
        #Si no hay nada devulve False
        except:
                return False
            
    def check_front_left(self):
        try:
            #Si hay algo delante a la izquierda devuleve True
            if(self.distances["frontal_left"] != 0):
                return True
            
            #Sino devuelve false
            else:
                return False
            
        #Si no hay nada devuelve False
        except:
                return False
    
    def check_central_right(self):
        try:
            #Si hay algo a la derecha devuelve True
            if(self.distances["central_right"] >= 2):
                return True
            
            #sino false
            else:
                return False
            
        #Si no hay nada devuelve False
        except:
            return False
    
    def check_central_left(self):
        try:
            
            #Si hay algo a la izquierda devuelve False
            if(self.distances["central_left"] >= 2):
                return True
            
            #Sino devuelve False
            else:
                return False
        
        #Si no hay nada devuelve false
        except:
            return False
       
    
    def get_points(self):
        # Obtenemos información sobre lo que hay delante del coche
        frontal = self.check_front()
        #Si hay algo en frontal
        if(frontal):
            self.points = self.points - 50
        else:
            self.points = self.points + 10
        
        # Obtenemos informacion sobre lo que hay detras
        back = self.check_back()
        
        #Si hay algo en back
        if(back):
            self.points = self.points - 50
        
        else:
            self.points = self.points + 10
            
        central_rigth = self.check_central_right()
        if(central_rigth):
            self.points = self.points - 50
        else:
            self.points = self.points + 10
        
        central_left = self.check_central_left()
        if(central_left):
            self.points = self.points - 50
        else:
            self.points = self.points + 10
        

    def __init__(self, dis):
        self.distances = dis
        self.points = 0
        self.get_points()
    

            
        
        

     