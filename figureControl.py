import matplotlib.pyplot as plt

class TextIncrementer:
    value: int
    textBox: plt.Text = None
	
    def __init__(self, value: int = 0) -> None:
        self.value = value
    
    def makeTextBox(self, axe: plt.Axes) -> None:
        self.textBox = axe.text(0.5, 0.5, str(self.value), fontsize=15, ha='center')
        plt.draw()
        plt.connect('key_press_event', self.update_text)

    def update_text(self, event):
        if event.key == 'up':
            self.value += 1
        elif event.key == 'down':
            self.value -= 1
        elif event.key == 'right':
            self.value += 10
        elif event.key == 'left':
            self.value -= 10
        elif event.key == 'escape':
            plt.close("all")
        self.textBox.set_text(str(self.value))
        plt.draw()

fig, axe = plt.subplots()

textInc = TextIncrementer(0)
textInc.makeTextBox(axe)

plt.show()