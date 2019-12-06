import rospy
from std_msgs.msg import String
def get_carpos(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return AddTwoIntsResponse(req.a + req.b)

def carpos_server():
    rospy.init_node('carpos_server')
    s = rospy.Service('carpos', String, get_carpos)
    print "Ready to get carpos."
    rospy.spin()

if __name__ == "__main__":
    carpos_server()