import pygame

# --- Init ---
pygame.init()
pygame.joystick.init()

# open first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

num_buttons = joystick.get_numbuttons()
num_axes = joystick.get_numaxes()

# --- Loop ---
clock = pygame.time.Clock()

while True:
    pygame.event.pump()  # updates internal joystick state

    # get button states
    buttons = [joystick.get_button(i) for i in range(num_buttons)]

    # get axis values (-1.0 to +1.0)
    axes = [round(joystick.get_axis(i), 2) for i in range(num_axes)]
    data=f"Buttons:{buttons} , Axes:{axes}"
    print(data)
    

    clock.tick(30)  # 30 Hz refresh (adds delay automatically)
