from __future__ import division
import socket
import time
# Import the PCA9685 module.
import Adafruit_PCA9685
import RPi.GPIO as GPIO
import cv2
import threading
import common as cm
# from human_follower2 import model,model_dir,lbl,track_object,top_k,threshold,arr_track_data,append_text_img1

pwm = Adafruit_PCA9685.PCA9685()

from PIL import Image


# Alternatively specify a different address and/or bus:

high_speed = 4000  # Max pulse length out of 4096
mid_speed = 2000  # Max pulse length out of 4096
low_speed = 1000  # Max pulse length out of 4096
short_delay = 0.1
long_delay = 0.2
extra_long_delay = 0.3

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)
GPIO.setmode(GPIO.BCM)  # GPIO number  in BCM mode
GPIO.setwarnings(False)


ob_range = 30  # minimum obstacle distance

# define ultrasonic sensor pins
GPIO_TRIGGER = 20
GPIO_ECHO = 21
servo_lft = 500  # ultrasonic sensor facing right
servo_ctr = 300  # ultrasonic sensor facing front
servo_rgt = 150  # ultrasonic sensor facing left

pwm.set_pwm(15, 0, servo_ctr)
time.sleep(3)  # servo facing front for 3s in order to make orientation alignment

# define L298N(Model-Pi motor drive board) GPIO pins
IN1 = 23  # left motor direction pin
IN2 = 24  # left motor direction pin
IN3 = 27  # right motor direction pin
IN4 = 22  # right motor direction pin
ENA = 0  # left motor speed PCA9685 port 0
ENB = 1  # right motor speed PCA9685 port 1
sensor1 = 5  # No.1 sensor from far left
sensor2 = 6  # No.2 sensor from left
sensor3 = 13  # middle sensor
sensor4 = 19  # No.2 sensor from right
sensor5 = 26  # No.1 sensor from far  right
sts1 = 0
sts2 = 0
sts3 = 0
sts4 = 0
sts5 = 0
cur_status = '0'

# Define motor control  pins as output
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(sensor1, GPIO.IN)
GPIO.setup(sensor2, GPIO.IN)
GPIO.setup(sensor3, GPIO.IN)
GPIO.setup(sensor4, GPIO.IN)
GPIO.setup(sensor5, GPIO.IN)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)  # Trigger
GPIO.setup(GPIO_ECHO, GPIO.IN)      # Echo
# Set trigger to False (Low)
GPIO.output(GPIO_TRIGGER, False)

speed_left = 2000
speed_right = 2000
motion_indicator = 0
speed_human=1200


threshold=0.2
top_k=5 
model_dir = '/home/pi/IoT/all_models'
model = 'mobilenet_ssd_v2_coco_quant_postprocess.tflite'
lbl = 'coco_labels.txt'

tolerance=0.1
x_deviation=0
y_max=0
arr_track_data=[0,0,0,0,0,0]

object_to_track='person'

from flask import Flask, Response
from flask import render_template

app = Flask(__name__)

@app.route('/')
def index():
    #return "Default Message"
    return render_template("index.html")

@app.route('/video_feed')
def video_feed():
    #global cap
    return Response(thread(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')
                    
def changespeed(speed_left, speed_right):
    pwm.set_pwm(ENA, 0, speed_left)
    pwm.set_pwm(ENB, 0, speed_right)



def stopcar():
    changespeed(0, 0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    


stopcar()


def backward(speed_left, speed_right,motion_indicator):
    changespeed(speed_left, speed_right)
    motion_indicator = 2
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    
    return motion_indicator

def forward1(speed_left, speed_right):
    changespeed(speed_left, speed_right)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)



def forward(speed_left, speed_right,motion_indicator):
    changespeed(speed_left, speed_right)
    motion_indicator = 1
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)

    return motion_indicator
    



def turnRight(speed_left, speed_right):
    changespeed(speed_left, speed_right)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
  

def turnLeft(speed_left, speed_right):
    changespeed(speed_left, speed_right)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    


def measure():
    # This function measures a distance
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    start = time.time()
    while GPIO.input(GPIO_ECHO) == 0:
        start = time.time()
    while GPIO.input(GPIO_ECHO) == 1:
        stop = time.time()
    elapsed = stop-start
    distance = (elapsed * 34300)/2
    return distance


def obstacle_avoid():
    pwm.set_pwm(15, 0, servo_lft)
    time.sleep(0.3)
    distance = measure()
    sts1 = 0 if distance > ob_range else 1

    pwm.set_pwm(15, 0, servo_ctr)
    time.sleep(0.3)
    distance = measure()
    sts2 = 0 if distance > ob_range else 1

    pwm.set_pwm(15, 0, servo_rgt)
    time.sleep(0.3)
    distance = measure()
    sts3 = 0 if distance > ob_range else 1
    sensorval = ''.join([str(sts1), str(sts2), str(sts3)])

    if sensorval == "100":
        print("100 slight right")
        forward(high_speed, mid_speed,motion_indicator)  # slight right turn
        time.sleep(long_delay)
        stopcar()
        time.sleep(short_delay)

    if sensorval == "001":
        print("001 slight left")
        forward(mid_speed, high_speed,motion_indicator)  # slight left turn
        time.sleep(long_delay)
        stopcar()
        time.sleep(short_delay)

    if sensorval == "110":
        print("110 sharp right")
        turnRight(high_speed, low_speed)  # shart right turn
        time.sleep(long_delay)
        stopcar()
        time.sleep(short_delay)

    if sensorval == "011" or sensorval == "010":
        print(sensorval+" sharp left")
        turnLeft(low_speed, high_speed)  # sharp left turn
        time.sleep(long_delay)
        stopcar()
        time.sleep(short_delay)

    if sensorval == "111" or sensorval == "101":
        print(sensorval+" back to left")
        turnRight(low_speed, high_speed)  # back to left side
        time.sleep(extra_long_delay)
        stopcar()
        time.sleep(short_delay)

    if sensorval == "000":
        print(sensorval+" forward")
        forward(mid_speed, mid_speed,motion_indicator)  # go forward
        time.sleep(long_delay)
        stopcar()
        time.sleep(short_delay)


def ticker(cur_status,motion_indicator):
    if cur_status == 'R':
        if(motion_indicator == 1): # in forward mode
            motion_indicator = forward(high_speed,low_speed,motion_indicator)
        elif(motion_indicator == 2): # in backward mode
            motion_indicator = backward(high_speed,low_speed,motion_indicator)
        else:
            turnRight(high_speed,0)
    if cur_status == 'L':
        if (motion_indicator == 1): #if currently in forward
            forward(low_speed,high_speed,motion_indicator)
        elif(motion_indicator ==2): # if in backward 
            backward(low_speed,high_speed,motion_indicator)
        else:
            turnLeft(0, high_speed)
    if cur_status == 'F':
        motion_indicator = forward(mid_speed,int(mid_speed*1.2),motion_indicator)
    if cur_status == 'B':
        motion_indicator = backward(mid_speed, int(mid_speed*1.1),motion_indicator)
    if cur_status == 'S':
        stopcar()
        motion_indicator = 0
    if cur_status == 'H':

        pass
    if cur_status == 'O':
        obstacle_avoid()
    
    return motion_indicator

def thread():
    cap = cv2.VideoCapture(0)
    global cur_status
    mdl=model   
    interpreter, labels =cm.load_model(model_dir,mdl,lbl,0)
    
    fps=10
    arr_dur=[0,0,0]

    while True:
        if cur_status !='H':
        #----------------Capture Camera Frame-----------------
            time.sleep(0.2)
            # start_t0=time.time()
            start_time=time.time()
            ret, frame = cap.read()
            if not ret:
                break
                
            cv2_im = frame
            cv2_im = cv2.flip(cv2_im, 0)
            cv2_im = cv2.flip(cv2_im, 1)

            
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
                
            ret, jpeg = cv2.imencode('.jpg', cv2_im)
            pic = jpeg.tobytes()
                
                #Flask streaming
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + pic + b'\r\n\r\n')
            
            # arr_dur[2]=time.time() - start_t2
            fps = round(1.0 / (time.time() - start_time),1)
            print("*********FPS: ",fps,"************")
        else:
            start_time = time.time()

        # ----------------Capture Camera Frame-----------------
            start_t0 = time.time()
            ret, frame = cap.read()
            if not ret:
                break

            cv2_im = frame
            cv2_im = cv2.flip(cv2_im, 0)
            cv2_im = cv2.flip(cv2_im, 1)

            cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
            pil_im = Image.fromarray(cv2_im_rgb)

            arr_dur[0] = time.time() - start_t0
            # ----------------------------------------------------

            # -------------------Inference---------------------------------
            start_t1 = time.time()
            cm.set_input(interpreter, pil_im)
            interpreter.invoke()
            objs = cm.get_output(interpreter, score_threshold=threshold, top_k=top_k)

            arr_dur[1] = time.time() - start_t1
            # ----------------------------------------------------

            # -----------------other------------------------------------
            start_t2 = time.time()
            track_object(objs, labels)  # tracking  <<<<<<<

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            cv2_im = append_text_img1(cv2_im, objs, labels, arr_dur, arr_track_data)
            # cv2.imshow('Object Tracking - TensorFlow Lite', cv2_im)

            ret, jpeg = cv2.imencode('.jpg', cv2_im)
            pic = jpeg.tobytes()

            # Flask streaming
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + pic + b'\r\n\r\n')

            arr_dur[2] = time.time() - start_t2
            fps = round(1.0 / (time.time() - start_time), 1)
            print("*********FPS: ", fps, "************")
            


    cap.release()
    cv2.destroyAllWindows()

def track_object(objs,labels):
   
    global x_deviation, y_max, tolerance, arr_track_data
    
    if(len(objs)==0):
        print("no objects to track")
        # stop()
        stopcar()
        # red_light("OFF")
        arr_track_data=[0,0,0,0,0,0]
        return
    
    flag=0
    for obj in objs:
        lbl=labels.get(obj.id, obj.id)
        if (lbl==object_to_track):
            x_min, y_min, x_max, y_max = list(obj.bbox)
            flag=1
            break
        
    #print(x_min, y_min, x_max, y_max)
    if(flag==0):
        print("selected object no present")
        return
        
    x_diff=x_max-x_min
    y_diff=y_max-y_min
    print("x_diff: ",round(x_diff,5))
    print("y_diff: ",round(y_diff,5))
        
        
    obj_x_center=x_min+(x_diff/2)
    obj_x_center=round(obj_x_center,3)
    
    obj_y_center=y_min+(y_diff/2)
    obj_y_center=round(obj_y_center,3)
    
    #print("[",obj_x_center, obj_y_center,"]")
        
    x_deviation=round(0.5-obj_x_center,3)
    y_max=round(y_max,3)
        
    print("{",x_deviation,y_max,"}")
   
    thread = threading.Thread(target = move_robot)
    thread.start()
    
    arr_track_data[0]=obj_x_center
    arr_track_data[1]=obj_y_center
    arr_track_data[2]=x_deviation
    arr_track_data[3]=y_max
    

def move_robot():
    global x_deviation, y_max, tolerance, arr_track_data
    
    print("moving robot .............!!!!!!!!!!!!!!")
    print(x_deviation, tolerance, arr_track_data)
    
    y=1-y_max #distance from bottom of the frame
    
    if(abs(x_deviation)<tolerance):
        delay1=0
        if(y<0.1):
            cmd="Stop"
            stopcar()
        else:
            cmd="forward"
            forward1(speed_human,int(speed_human*1.15))
    
    else:
        if(x_deviation>=tolerance):
            cmd="Move Left"
            delay1=get_delay(x_deviation)
                
            turnLeft(speed_human,speed_human)
            time.sleep(delay1)

            stopcar()
                
        if(x_deviation<=-1*tolerance):
            time.sleep(0.2)
            cmd="Move Right"
            delay1=get_delay(x_deviation)
                
 
            turnRight(speed_human,speed_human)
            time.sleep(delay1)

            stopcar()

    arr_track_data[4]=cmd
    arr_track_data[5]=delay1

def get_delay(deviation):
    deviation=abs(deviation)
    if(deviation>=0.4):
        d=0.080
    elif(deviation>=0.35 and deviation<0.40):
        d=0.060
    elif(deviation>=0.20 and deviation<0.35):
        d=0.050
    else:
        d=0.040
    return d
    

def main():

    try:
        s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        s.connect(('8.8.8.8',80))
        ip =s.getsockname()[0]
    finally:
        s.close()
    print(ip)
    #-----------------------------------
    PORT = 8000

    sock = socket.socket(socket.AF_INET,  # Internet
                        socket.SOCK_STREAM)  # TCP
    sock.bind((ip, PORT))
    sock.listen(4)
    print()
    # t2=threading.Thread(target=thread_2)
    t1=threading.Thread(target=thread)
        # if not t1.is_alive:
    # t2.start()
    

    global cur_status,motion_indicator
    # arr_dur=[0,0,0]
    while True:
        print(cur_status)
        if not t1.is_alive:
            t1.start()
        
        print("waiting orders")
        sock.settimeout(0.5)
        print("motion_indicator =",motion_indicator)
        try:
            s,addr = sock.accept()
            print("from",addr)
            data = s.recv(128)
            print("data=",data)
        except socket.timeout:
            ticker(cur_status, motion_indicator)
            continue
        if data == b'R': # turn right
            cur_status = 'R'
            motion_indicator = ticker(cur_status,motion_indicator)

        if data == b'L':
            cur_status = 'L'
            motion_indicator = ticker(cur_status,motion_indicator)

        if data == b'F':
            cur_status = 'F'
            motion_indicator = ticker(cur_status,motion_indicator)

        if data == b'B':
            cur_status = 'B'
            motion_indicator = ticker(cur_status,motion_indicator)

        if data == b'S': # stop 
            cur_status = 'S'
            motion_indicator = ticker(cur_status,motion_indicator)

        if data == b'E':
            cur_status = 'E'
            motion_indicator = ticker(cur_status,motion_indicator)

        if data == b'O':
            cur_status = 'O'
            motion_indicator = ticker(cur_status,motion_indicator)

        if data == b'H':
            cur_status = 'H'
            motion_indicator = ticker(cur_status,motion_indicator)
    
        
def append_text_img1(cv2_im, objs, labels, arr_dur, arr_track_data):
    height, width, channels = cv2_im.shape
    font=cv2.FONT_HERSHEY_SIMPLEX
    
    global tolerance
    
    #draw black rectangle on top
    cv2_im = cv2.rectangle(cv2_im, (0,0), (width, 24), (0,0,0), -1)
   
    #write processing durations
    cam=round(arr_dur[0]*1000,0)
    inference=round(arr_dur[1]*1000,0)
    other=round(arr_dur[2]*1000,0)
    text_dur = 'Camera: {}ms   Inference: {}ms   other: {}ms'.format(cam,inference,other)
    # cv2_im = cv2.putText(cv2_im, text_dur, (int(width/4)-30, 16),font, 0.4, (255, 255, 255), 1)
    
    #write FPS 
    total_duration=cam+inference+other
    fps=round(1000/total_duration,1)
    text1 = 'FPS: {}'.format(fps)
    cv2_im = cv2.putText(cv2_im, text1, (10, 20),font, 0.7, (150, 150, 255), 2)
   
    
    #draw black rectangle at bottom
    cv2_im = cv2.rectangle(cv2_im, (0,height-24), (width, height), (0,0,0), -1)
    
    #write deviations and tolerance
    str_tol='Tol : {}'.format(tolerance)
    cv2_im = cv2.putText(cv2_im, str_tol, (10, height-8),font, 0.55, (150, 150, 255), 2)
  
    x_dev=arr_track_data[2]
    str_x='X: {}'.format(x_dev)
    if(abs(x_dev)<tolerance):
        color_x=(0,255,0)
    else:
        color_x=(0,0,255)
    cv2_im = cv2.putText(cv2_im, str_x, (110, height-8),font, 0.55, color_x, 2)
    
    y_dev=arr_track_data[3]
    str_y='Y: {}'.format(y_dev)
    if(abs(y_dev)>0.9):
        color_y=(0,255,0)
    else:
        color_y=(0,0,255)
    cv2_im = cv2.putText(cv2_im, str_y, (220, height-8),font, 0.55, color_y, 2)
   
    #write command, tracking status and speed
    cmd=arr_track_data[4]
    cv2_im = cv2.putText(cv2_im, str(cmd), (int(width/2) + 10, height-8),font, 0.68, (0, 255, 255), 2)
    
    delay1=arr_track_data[5]
    str_sp='Speed: {}%'.format(round(delay1/(0.1)*100,1))
    cv2_im = cv2.putText(cv2_im, str_sp, (int(width/2) + 185, height-8),font, 0.55, (150, 150, 255), 2)
    
    if(cmd==0):
        str1="No object"
    elif(cmd=='Stop'):
        str1='Acquired'
    else:
        str1='Tracking'
    cv2_im = cv2.putText(cv2_im, str1, (width-140, 18),font, 0.7, (0, 255, 255), 2)
    
    #draw the center red dot on the object
    cv2_im = cv2.circle(cv2_im, (int(arr_track_data[0]*width),int(arr_track_data[1]*height)), 7, (0,0,255), -1)

    #draw the tolerance box
    cv2_im = cv2.rectangle(cv2_im, (int(width/2-tolerance*width),0), (int(width/2+tolerance*width),height), (0,255,0), 2)
    
    for obj in objs:
        x0, y0, x1, y1 = list(obj.bbox)
        x0, y0, x1, y1 = int(x0*width), int(y0*height), int(x1*width), int(y1*height)
        percent = int(100 * obj.score)
        
        box_color, text_color, thickness=(0,150,255), (0,255,0),1
        

        text3 = '{}% {}'.format(percent, labels.get(obj.id, obj.id))
        
        if(labels.get(obj.id, obj.id)=="person"):
            cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), box_color, thickness)
            cv2_im = cv2.putText(cv2_im, text3, (x0, y1-5),font, 0.5, text_color, thickness)
            
    return cv2_im            
        
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=2204, threaded=True) # Run FLASK
    main()
    