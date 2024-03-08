import numpy as np
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from tkinter import *
from tkcalendar import DateEntry

class ROSNode:
    def __init__(self, master):
        self.master = master
        self.setup_ros()
        self.setup_gui()
        self.position = [None, None, None, None]

    def setup_ros(self):
        rospy.init_node('gazebo_model_state_publisher')
        self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        
    def callback(self, model_states):
        t = rospy.get_rostime().to_sec()
        for model_name, pose, twist in zip(model_states.name, model_states.pose, model_states.twist):
            if model_name == 'atom':
                self.position=[int(pose.orientation.x), int(pose.orientation.y), int(pose.orientation.z), int(pose.orientation.w)]
                print(self.position)
                # Burada orientation verileri ile bir şeyler yapabilirsiniz.

    def setup_gui(self):
        self.canvas = Canvas(self.master, height=800, width=1200)
        self.canvas.pack()
        
        self.frame_ust = Frame(self.master, bg='#808080')
        self.frame_ust.place(relx=0.1, rely=0.1, relwidth=0.8, relheight=0.1)
        
        self.frame_sol_alt = Frame(self.master, bg='#808080')
        self.frame_sol_alt.place(relx=0.1, rely=0.2, relwidth=0.3, relheight=0.4)
        
        # frame_sag_alt oluşturuluyor
        self.frame_sag_alt = Frame(self.master, bg='#808080')
        self.frame_sag_alt.place(relx=0.46, rely=0.2, relwidth=0.4, relheight=0.4)

        # frame_new_ros oluşturuluyor
        self.frame_new_ros = Frame(self.master, bg='#808080')
        self.frame_new_ros.place(relx=0.1, rely=0.7, relwidth=0.8, relheight=0.35)
        
        Label(self.frame_new_ros, text="ROS MESAJ:", bg="#808080", font="Ariel 12 bold").pack(padx=10, pady=10, anchor=N)
        Label(self.frame_sag_alt, text="dırımdırım",bg="#808080",font= "Ariel 12 bold").pack(padx=10,pady=10, anchor=N)
        Label(self.frame_new_ros, text=self.position, bg="#808080", font="Ariel 20 bold").pack(padx=10, pady=20, anchor=N)
        var = IntVar()

        R1 = Radiobutton(self.frame_sol_alt, text="Sisteme Kaydet", variable = var, value=1 , bg='#808080', font='Ariel 12 bold')
        R1.pack(anchor=NW, padx=5,pady=15)

        R2 = Radiobutton(self.frame_sol_alt, text="E-posta Gönder", variable = var, value=2 , bg='#808080', font='Ariel 12 bold')
        R2.pack(anchor=NW, padx=5,pady=5)

        var1 = IntVar()
        C1 = Checkbutton(self.frame_sol_alt, text="Bir hafta önce", variable=var1, onvalue=1, offvalue=0, bg='#808080', font='Ariel 10 bold')
        C1.pack(padx=25,pady=5, anchor=NW)

        var2 = IntVar()
        C1 = Checkbutton(self.frame_sol_alt, text="Bir gün önce", variable=var2, onvalue=1, offvalue=0, bg='#808080', font='Ariel 10 bold')
        C1.pack(padx=25,pady=5, anchor=NW)

        var3 = IntVar()
        C1 = Checkbutton(self.frame_sol_alt, text="Aynı Gün", variable=var3, onvalue=1, offvalue=0, bg='#808080', font='Ariel 10 bold')
        C1.pack(padx=25,pady=5, anchor=NW)

        metin_alani = Text(self.frame_sag_alt,bg='white')
        metin_alani.place(relx=0.05, rely=0.2, relheight=0.6, relwidth=0.9)
        metin_alani.tag_configure('style', foreground= '#bfbfbf', font=('verdana',7,'bold'))

        karsilama_metni = 'Mesajını buraya gir...'
        metin_alani.insert(END,karsilama_metni, 'style')

        gonder_butonu = Button(self.frame_sag_alt, text="Gönder", command=self.gonder)
        gonder_butonu.place(relx=0.40, rely=0.85)

    def gonder(self):
        pass
        return
        
    
               

if __name__ == '__main__':
    try:
        master = Tk()
        app = ROSNode(master)
        master.mainloop()
        #ROSNode.setup_ros()
    except Exception as e:
        rospy.logfatal("gazebo_model_state_publisher died")
        print(str(e))
