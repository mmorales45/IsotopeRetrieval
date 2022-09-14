#!/usr/bin/env python

import rospy
import smach

# define navigating state
class Navigate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define manipulation state
class Manipulate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome2'

        



# main
def main():
    rospy.init_node('argonne_SMACH')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Navigate', Navigate(), 
                               transitions={'outcome1':'Manipulate'})

        smach.StateMachine.add('Manipulate', Manipulate(), 
                               transitions={'outcome2':'outcome4'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
