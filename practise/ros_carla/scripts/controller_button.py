import time

class Button:
    # Constructor   
    def __init__(self):
        self.date_of_pushed=self.date_of_released=[]
        self.is_the_button_pressed=False

    # Set the state of the button.
    def set_state_of_button(self,bool_set):
        self.is_the_button_pressed=bool_set
    
    # This function is used for reseting the time.
    def set_date_of_pushed_and_released(self,list):
        self.date_of_pushed=self.date_of_released=list
    
    # Function that returns that the button is pressed.
    def get_state_of_button(self):
        return self.is_the_button_pressed

    # Function that returns the time when the button is pushed.
    def get_when_the_button_is_pushed(self):
        return self.date_of_pushed

    # Function that returns the time when the button is released. 
    def get_when_the_button_is_released(self):
        return self.date_of_released

    # Function that sets the time when button is pushed.
    def add_pushed_time(self):
        self.date_of_pushed.append(time.strftime("%H"))
        self.date_of_pushed.append(time.strftime("%M"))
        self.date_of_pushed.append(time.strftime("%S"))

    # Function that sets the time when button is released.
    def add_released_time(self):
        self.date_of_released.append(time.strftime("%H"))
        self.date_of_released.append(time.strftime("%M"))
        self.date_of_released.append(time.strftime("%S"))

# pak ne raboti towa....