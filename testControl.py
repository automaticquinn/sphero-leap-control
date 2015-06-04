import rospy
import wx
from sphero_driver import sphero_driver
from std_msgs.msg import String

class D: pass
D.vel = 0
D.head = 0
D.color =0
D.hand = False
  
class MyForm(wx.Frame):
    global robobot
    
  
    def fun( self, data ):
        message = data.data
        #print "I received the string", message,

        N = len("data")
        m = message[N:]
        #print "m is", m
        T = eval( m )
        D.head = T[0]
        D.vel = T[1]
        D.color = int(T[2])
        print "Data is", D.head, D.vel, D.color
        robobot.set_stablization(True, False)
        robobot.set_back_led(200, False)
        if(D.vel%120 >20):
            robobot.roll(int(D.vel%120), int(D.head%259), 1, False)
        else:
            robobot.roll(0, int(D.head%259), 1, False)

        if(D.color < 85):
            robobot.set_rgb_led(0,0, D.color, False ,False)
        elif(100 > D.color > 50 ):
            robobot.set_rgb_led(D.color, 0, 0, False,False)
        elif(200 > D.color > 100):
            robobot.set_rgb_led(D.color, D.color, D.color, False,False)
        elif(D.color > 200):
            robobot.set_rgb_led(D.color, 0, 0, False,False)


        # if / elif//
  
    def __init__(self):
        global robobot
        robobot = sphero_driver.Sphero()
        robobot.connect()

        rospy.init_node('keyboard')

        rospy.Subscriber( 'text_data', String, self.fun )
  
        wx.Frame.__init__(self, None, wx.ID_ANY, "Key Press Tutorial")
  
        # Add a panel so it looks the correct on all platforms
        panel = wx.Panel(self, wx.ID_ANY)
        btn = wx.Button(panel, label="OK")
  
        btn.Bind(wx.EVT_KEY_DOWN, self.onKeyPress)

        
  
    def onKeyPress(self, event):
        keycode = event.GetKeyCode()
        # speed = 20
        # heading = 0
        # print keycode
        if keycode == wx.WXK_DOWN:
            robobot.roll(0, int(D.head%259), 1, False)
        #     print "you pressed up!"
        #     robobot.roll(speed, 0, 1, False)
        # elif keycode == wx.WXK_DOWN:
        #     print "you pressed down!"
        #     robobot.roll(speed, 180, 1, False)
        # elif keycode == wx.WXK_LEFT:
        #     print "you pressed left!"
        #     robobot.roll(speed, 270, 1, False)
        # elif keycode == wx.WXK_RIGHT:
        #     print "you pressed right!"
        #     robobot.roll(speed, 90, 1, False)
        event.Skip()
  
# Run the program
if __name__ == "__main__":
    app = wx.PySimpleApp()
    frame = MyForm()
    frame.Show()
    app.MainLoop()
    