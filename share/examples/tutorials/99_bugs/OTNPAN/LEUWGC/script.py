

def execute(self, inputs, outputs, gvm):
    from random import random, randint
    if random() < 0.9:
        outputs["Bugs"] = inputs["Bugs"] - 1
    else:
        outputs["Bugs"] = inputs["Bugs"] + randint(15, 35)
    return 0

