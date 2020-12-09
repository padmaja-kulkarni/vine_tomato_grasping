

def initialize_drop_down_button(button, options, cb):
    """
        initializes a drop down button
    """
    button.clear()
    button.setEditable(True)
    for option in options:
        button.addItem(option)

    button.activated.connect(cb)
    return button