from attachment_template import *

child = ChildController()
carer = CarerController()

for i in range(100):
    if child.currentAction() == Action.APPROACH:
        print('(Child) Doing approach')
        if( careGiven()):
            child.updateEmotionalDistance()
            receiveCare()
    else:
        print('(Child) Doing explore')

    if carer.currentAction() == Action.APPROACH:
        print('(Carer) Doing approach')

        if( isClose() ):
            giveCare()
            carer.updateEmotionalDistance()
    else:
        print('(Carer) Doing explore')

    
    child.updatePhysicalDistance()
    carer.updatePhysicalDistance()
    
