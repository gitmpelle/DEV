##### import GenSetModbus_rev106
from esp32 import *
import sys
import os
#from umqttsimple import MQTTClient
from simple2 import MQTTClient
from machine import Pin, UART, PWM, WDT, Timer, ADC, DAC
import machine, neopixel
from uModBusSerial import uModBusSerial
from time import sleep
import time
import utime
# import uping
import ubinascii
import machine
import micropython
import network, ntptime
from ntptime import settime
import esp
esp.osdebug(None)
import json
import ujson
from collections import OrderedDict
import ntptime
rtc = machine.RTC()
from time import sleep_ms
import ubluetooth
# import ds18x20
# import onewire
import gc
gc.collect()

# os.dupterm(uart, 1)
#datatest = [2533, 2537, 2529, 2536, 2548, 2534, 2529, 2536, 2544, 2530, 2547, 2543, 2543, 2532, 2545, 2528, 2545, 2539, 2538, 2543, -8828, 1000, 0, 1000, 31726, 20, 488, -14768, 5939, 2538]
#data1 = [3533, 2537, 2529, 2536, 2548, 2534, 2529, 2536, 2544, 2530, 2547, 2543, 2543, 2532, 2545, 2528, 2545, 2539, 2538, 2543, -8828, 1000, 0, 1000, 31726, 20, 488, -14768, 5939, 2538]
#mydata = '{"TIME":"2022-07-21 22:22:19","primary_relay":1,"secondary_relay":0,"Key_Sw_Sns":1,"core_temperature":25.172,"status":"0000000111100000","CELL12":2.323,"CELL13":2.323,"CELL10":2.324,"CELL11":2.324,"CELL16":2.323,"CELL17":2.327,"CELL03":2.323,"CELL02":2.324,"CELL01":2.323,"CELL14":2.323,"CELL07":2.323,"CELL06":2.323,"CELL05":2.327,"CELL04":2.323,"CELL15":2.323,"CELL18":2.323,"CELL09":2.323,"CELL08":2.335,"CELL19":2.323,"CELL20":2.323,"topv":46.607,"Sum":-19.047,"topv_sum_diff":0.117,"Avg":2.324}'
#bulk_current = [8,15,22,30]
bulk_current = [30,60,90,120]
#bulk_current = [15,30,45,60]
bulk_bit =      [0,0,0,0]
bulk_current_set = 0
kCell_down = 0
stack_capacity = 0
Circuit_breakers = 0
SOC = 0
wdt = WDT(timeout=8000)
reg_800 = [] 
mq_broker = ''
mq_user = ''
mq_pwd = ''
mq_port = ''
mq_topic = ''
mq_subscribe = ''
wifi_ssid = ''
wifi_pwd = ''
wifi_en = '' 
account_id = ''
board_id = ''
Master_status = 0

class MyException(Exception):
    pass


def load_config():
    global  mq_broker 
    global  mq_user
    global  mq_pwd
    global  mq_port
    global  mq_topic
    global  mq_subscribe
    global  wifi_ssid
    global  wifi_pwd
    global  wifi_en
    global config
    global account_id
    global board_id
    try:
        
        with open("config.json") as json_file:
            config = ujson.load(json_file)
            print(f"IN: {config}")        

        mq_broker = config["mq_broker"]
        mq_user = config["mq_user"]
        mq_pwd = config["mq_pwd"]
        mq_port = config["mq_port"]
        mq_topic = config["mq_topic"]
        mq_subscribe = config["mq_subscribe"]
        wifi_ssid = config["wifi_ssid"]
        wifi_pwd = config["wifi_pwd"]
        wifi_en = int(config["wifi_en"])
        account_id = config["account_id"]
        board_id = config["board_id"] 
        print(mq_subscribe)
    except Exception as e:
        print(f"ERROR: load_config {e}")
        pass


try:
    s = machine.unique_id()
    with open('config.json') as json_file:
        data = ujson.load(json_file)
#         print(f'IN: {data}')
    data['board_id'] = int("".join(map(str, s)))
    with open('config.json', 'w') as outfile:
        ujson.dump(data, outfile)
    print(f'done update config')
except Exception as e:
    print(f'ERROR: ID load_config {e}')
    pass


load_config()

# Log file section
##*******************
#we will use the inbuilt LED as a file activity light
LED_FileWrite = machine.Pin(2,machine.Pin.OUT)
#Define a file name and a maximum size in bytes
# LogFileName = "log.txt"
Max_File_Size = 100000
     
#This writes whatever is passed to it to the file     
def WriteFile(passed):
    global LogFileName
    print(f'saving offline...')
    LED_FileWrite.value(1) #indicate writing to file, so don't power off
    log = open('/log/'+LogFileName,'a') #open in append - creates if not existing, will append if it exists
    log.write(passed)
    log.close()
    
    LED_FileWrite.value(0)

#This returns the size of the file, or 0 if the file does not exist
def CheckFileSize():
    # f is a file-like object.
    try:
        f = open('/log/'+LogFileName,"r") # Open read - this throws an error if file does not exist - in that case the size is 0
        f.seek(0, 2)
        size = f.tell()
        f.close()
        return size
    except:
        return 0
        pass 

def sendOffline():
    global mq_topic, account_id, board_id, mq_subscribe
    idint = 'bugbug'
    data = []
    count = 0
    message2 = f"masters/{board_id}"
    try:
        list_of_files = os.listdir('log')
        for x in list_of_files:
            if list_of_files:
                msg = f'/log/{x}'
                file1 = open(msg, 'r')
                print(f'sending... {x}')
                while True:
                    wdt.feed()
                    count += 1
                    # Get next line from file
                    line = file1.readline()
                    if not line:
                        break            
                    data = line
                    client.publish(message2, data)
            os.remove(msg)
            time.sleep(3)
        return
    except  Exception as e:
      print(f'sendOffline Error... {count} {e}')
      pass
#We will just write an incrementing number to the file
number = 0


##********************
ble_msg = ""
is_ble_connected = False
#####******************************************************************************
# This example demonstrates a peripheral implementing the Nordic UART Service (NUS).
import bluetooth
from ble_advertising import advertising_payload

from micropython import const


_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)

_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX = (
    bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"),
    _FLAG_NOTIFY,
)
_UART_RX = (
    bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"),
    _FLAG_WRITE,
)
_UART_SERVICE = (
    _UART_UUID,
    (_UART_TX, _UART_RX),
)

# org.bluetooth.characteristic.gap.appearance.xml
_ADV_APPEARANCE_GENERIC_COMPUTER = const(128)


class BLEUART:
    def __init__(self, ble, name="master "+ str(account_id), rxbuf=100):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._tx_handle, self._rx_handle),) = self._ble.gatts_register_services((_UART_SERVICE,))
        # Increase the size of the rx buffer and enable append mode.
        self._ble.gatts_set_buffer(self._rx_handle, rxbuf, True)
        self._connections = set()
        self._rx_buffer = bytearray()
        self._handler = None
        # Optionally add services=[_UART_UUID], but this is likely to make the payload too large.
        self._payload = advertising_payload(name=name, appearance=_ADV_APPEARANCE_GENERIC_COMPUTER)
        self._advertise()

    def irq(self, handler):
        self._handler = handler

    def _irq(self, event, data):
        # Track connections so we can send notifications.
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            if conn_handle in self._connections:
                self._connections.remove(conn_handle)
            # Start advertising again to allow a new connection.
            self._advertise()
        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            if conn_handle in self._connections and value_handle == self._rx_handle:
                self._rx_buffer += self._ble.gatts_read(self._rx_handle)
                if self._handler:
                    self._handler()

    def any(self):
        return len(self._rx_buffer)

    def read(self, sz=None):
        if not sz:
            sz = len(self._rx_buffer)
        result = self._rx_buffer[0:sz]
        self._rx_buffer = self._rx_buffer[sz:]
        return result

    def write(self, data):
        for conn_handle in self._connections:
            self._ble.gatts_notify(conn_handle, self._tx_handle, data  + "\n")

    def close(self):
        for conn_handle in self._connections:
            self._ble.gap_disconnect(conn_handle)
        self._connections.clear()

    def _advertise(self, interval_us=500000):
        self._ble.gap_advertise(interval_us, adv_data=self._payload)

ble_ = bluetooth.BLE()
ble = BLEUART(ble_)

def on_rx():
    global ble_msg
    ble_msg = ble.read().decode().strip()
    print("rx: ", ble_msg)

ble.irq(handler=on_rx)
    
####*******************************************************************************************************
#setup i2c port
sda=machine.Pin(21)
scl=machine.Pin(22)
i2c=machine.I2C(1,sda=sda, scl=scl, freq=10000)
devices = i2c.scan()
# print('*i2c devices found:',len(devices))

# dac = DAC(Pin(25)) # create DAC object on pin 25
# dac.write(128) # set the output voltage to 1.65V
# 
# AD0 = ADC(Pin(36)) #VP
# AD1 = ADC(Pin(39)) #VN
# AD2 = ADC(Pin(34))
# AD3 = ADC(Pin(35))
# AD4 = ADC(Pin(32))
# AD5 = ADC(Pin(33))
# #TopV_Pbus = ADC(Pin(25))
# 
# AD0.atten(ADC.ATTN_11DB)
# AD1.atten(ADC.ATTN_11DB)
# AD2.atten(ADC.ATTN_11DB)
# AD3.atten(ADC.ATTN_11DB)
# AD4.atten(ADC.ATTN_11DB)
# AD5.atten(ADC.ATTN_11DB)
# #TopV_Pbus.atten(ADC.ATTN_11DB)
# 
Key_Sw_Sns =   Pin(4,   Pin.IN, Pin.PULL_UP)
# Gen_Pwr_Det =  Pin(15,  Pin.IN, Pin.PULL_UP)
# SP_Power_Snse = Pin(23,  Pin.IN, Pin.PULL_UP)
# 
# 
_4v0_EN =   Pin(13, Pin.OUT)
I2C_Reset = Pin(19, Pin.OUT)
Pri_Relay = Pin(26, Pin.OUT)
Sec_Relay = Pin(27, Pin.OUT)
HS315_Alarm = Pin(14, Pin.OUT)
# #led = Pin(2, Pin.OUT)
# ds_pin = Pin(18, Pin.PULL_UP)


Pri_Relay.value(1)
Sec_Relay.value(1)
HS315_Alarm.value(1)
#led.value(0)
I2C_Reset.value(1)
_4v0_EN.value(1)







timer = Timer(0)
#timer.init(period=1000, mode=Timer.PERIODIC, callback=lambda t:led.value(not led.value()))
#timer.init(period=5000, mode=Timer.PERIODIC, callback=lambda t:led.value(not led.value()))

RCS = []
RIS = []
RIR = []
RHR = []
WSC = []
WSR = []
WMC = []
WMR = []
ADCs = [36,39,34,35,32,33]
ADCv = []
online = [0,0,0,0,0,0]
maxCell = [0,0,0,0,0,0]
topv = [0,0,0,0,0,0]
core = [0,0,0,0,0,0]
kCell_relay = [1,1,1,1,1,1]
kCell_topv = [0,1,1,1,1,1]
kCell_reply = [0,0,0,0,0,0]
cell_top = 0
Load_voltage = 0
Engine_battery = 0
'''
STAT_IDLE – no connection, no activities-1000
STAT_CONNECTING – Connecting-1001
STAT_WRONG_PASSWORD – Failed due to password error-202
STAT_NO_AP_FOUND – Failed, because there is no access point reply,201
STAT_GOT_IP – Connected-1010
STAT_ASSOC_FAIL – 203
STAT_BEACON_TIMEOUT – Timeout-200
STAT_HANDSHAKE_TIMEOUT – Handshake timeout-204
'''

# dictionary that maps string date names to indexes in the RTC's datetime tuple
# DATETIME_ELEMENTS = {
#     "year": 0,
#     "month": 1,
#     "day": 2,
#     "day_of_week": 3,
#     "hour": 4,
#     "minute": 5,
#     "second": 6,
#     "millisecond": 7
# }

# set an element of the RTC's datetime to a different value
def set_datetime_element(rtc, datetime_element, value):
    date = list(rtc.datetime())
    date[DATETIME_ELEMENTS[datetime_element]] = value
    rtc.datetime(date)

# create a blank log file / open it , if already present
#logFile = open("data_log.txt", "a")

last_message = 0
message_interval = 10
counter = 0

def getTime():
        timestamp=rtc.datetime()
        timestring="%04d-%02d-%02d %02d:%02d:%02d"%(timestamp[0:3] +  timestamp[4:7])
        return f'{timestring[0:20]}'

wlan=network.WLAN(network.STA_IF)
#WiFi.mode(WIFI_STA); // SETS TO STATION MODE!
def connectWiFi(ID,password):
  global wifi_en
  try:
      if not wifi_en:
          return False
      i=0
      wlan.active(True)
      wlan.disconnect()
      wlan.connect(ID, password)
      while(wlan.ifconfig()[0]=='0.0.0.0'):
        wdt.feed()  
        i=i+1
        print(f'connecting WiFi Countdown: {20-i} s')
        time.sleep(1)
        if(i>20):
          break
      if(i<20):
        print(f'WIFI connected')
        wdt.feed()
        settime()
        wdt.feed()
      else:
        print(f'NOT connected!',0,16)
      time.sleep(3)
      wdt.feed()
      return True
  except Exception as e:
        print(f"connectWiFi ERROR: {e}")
        return False
        pass
def sub_cb(topic, msg, r, d):

  global RCS
  global RIS
  global RIR
  global RHR
  global WSC
  global WSR
  global WMC
  global WMR
  global slaveID
  global function
  global register
  global data
  global count
  global rtu_flag
  global cal_flag
  

  print(f'receive: {msg}  topic: {topic}  ')
  rcv = json.loads(msg)
  
  try:
        print(f'currtime: {getTime()}') 
  except Exception as e:
        print(f"GET TIME ERROR: {e}")
        pass        
        
  if rcv["cmd"] == "mqtt_rtu":
      
         slave    = 0
         function = 0
         register = 0
         data  = 0
         qty   = 0
         key1 = 'data'
         key2 =  'qty'
         
         slave    = rcv['id']
         function = rcv['func']
         register = rcv['reg']
         if key1 in rcv:
            data  = rcv['data']
         if key2 in rcv:
            qty  =  rcv['qty']   
  
 # 1 Read Coil Status 2000
  if function == 1:
     RCS = [slave, register,qty]
#      print(f'RCS-1: {RCS}')
 # 2 Read Input Status 2000
  if function == 2:
     RIS = [slave, register,qty]
#      print(f'RIS-2: {RIS}')
 # 3 Read Holding Registers 125
  if function == 3:
     RHR = [slave, register,qty]
     print(f'RHR-3: {RHR}')
 # 4 Read Input Registers 125
  if function == 4:
     RIR = [slave, register,qty]
     print(f'RIR-4: {RIR}')
 # 5 Write Single Coil 1    
  if function == 5:
     WSC = [slave, register,data]
#      print(f'WSC-5: {WSC}')
 # 6 Write Single Register 1    
  if function == 6:
     WSR = [slave, register,data]
#      print(f'WSR-6: {WSR}')
 # 15 Write Multiple Coils 800
  if function == 15:
     WMC = [slave, register,data]
#      print(f'WMC-15: {WMC}')
 # 16 Write Multiple Registers    
  if function == 16:
     WMR = [slave, register,data]
#      print(f'WMR-16: {WMR}')    
 #17 READ_WRITE_MULTIPLE_REGISTERS
  if function == 17:
     RWMR = [slave, register,data]
#      print(f'RWMR-17: {RWMR}')  
 

def connect_and_subscribe():
  global  mq_broker, mq_topic, mq_user, mq_pwd, mq_port, mq_subscribe, client_id, account_id
  client = MQTTClient(client_id, mq_broker, user=mq_user, password=mq_pwd, port=mq_port)
  client.set_callback(sub_cb)
  client.connect()
  client.subscribe("masters/" + str(board_id) + '/rx', 0)
  print(f'subscribed to: masters/{str(board_id)}/rx')
  #print(mq_subscribe + str(board_id))
  #print(f'subscribed to: {mq_subscribe}{str(account_id)}')
  return client

def restart_and_reconnect():
#   print('Reconnecting...')
  time.sleep(1)
  connectWiFi(wifi_ssid,wifi_pwd)
  print(f'wifi_ssid:wifi_pwd {wifi_ssid} : {wifi_pwd}')
  time.sleep(5)
  if wlan.isconnected():
        client_id = ubinascii.hexlify(machine.unique_id())
        client = connect_and_subscribe()
  
def sendRtuMsg(msg):
     try:
         wdt.feed()
         s = machine.unique_id()
         idint = "".join(map(str, s))
         rtu_msg = "masters/" + str(board_id)
         curr_time = 'time.localtime()'
         currtime = getTime()
         message = {
                'TIME':currtime,
                'msgType' : "Debug",
                'rtuMsg' : msg
                }
         payload = json.dumps(message)
         if wlan.isconnected() :
             print(f' sendRtuMsg send: ')
             client.publish(rtu_msg, payload)
    
     except Exception as e:
            print(f' sendRtuMsg Error: {e}')
            pass
       
def sendmessage1():
     try:
         wdt.feed()
         s = machine.unique_id()
         idint = "".join(map(str, s))
         topic = "masters/" + str(idint)
         wdt.feed()
         currtime = getTime()
         wdt.feed()
         message = {
                'TIME':currtime,
                'msgType' : "hs315",
                "MASTER_ID": idint,
                'Gen_voltage':Gen_voltage,
                'Gen_current':Gen_current,
                'Load_voltage':Load_voltage,
                'Load_current':Load_current,
                'GCB_voltage':GCB_voltage,
                'Engine_coolant':Engine_coolant,
                'Load_watts':Load_watts,
                'Gen_watts':Gen_watts,                
                'Engine_rpm':Engine_rpm,
                'Engine_hours':Engine_hours,
                'Engine_battery':Engine_battery,
                'Engine_oil':Engine_oil,
                'Engine_status':Engine_status,
                'Controller_mode':Controller_mode,
                'Circuit_breakers':Circuit_breakers,
                'Battery_charge':Battery_charge,
                'Battery_status':Battery_status,
                'Battery_temperature':Battery_temperature,
                'Speed_governor_pct':Speed_governor_pct,
                'Current_charge':Current_charge,
                'SOC':SOC,
                'End_discharge':End_discharge,
                'End_charge':End_charge,
                'Discharge_time':Discharge_time,
                'Last_Discharge_time':Last_Discharge_time,
                'Engine_run_time':Engine_run_time,
                'Engine_fuel_used':Engine_fuel_used,
                'Charge_Cycles' :Charge_Cycles,
                'Discharge_Cycles' :Discharge_Cycles,
                "Master_status": reg_800,
                "kCell_Status": online[2:6], 
                "Status_bar" : Status_bar
                }
         payload = json.dumps(message)

         if wlan.isconnected() :
             print(f'publishing hs315 message')
             try:
                  wdt.feed()
                  print(f'publish: {client.publish(topic, payload)}')
                  wdt.feed()
                
                  
             except Exception as e:
                  print(f'*publishing hs315 Error: {e}')
                  pass
         return True
     except Exception as e:
            print(f'publishing hs315 Error: {e}')
            pass

def sendmessage4(online,maxCell,topv,topDiff,core,kCell_relay,kCell_topv,kCell_reply,kCell_down, stack_capacity, Circuit_breakers, SOC):
#print(f" Online: {online[2:6]} maxCell: {maxCell[2:6]} Topv: {topv[2:6]} topvDiff: {topDiff}")    
     global mq_topic, account_id, board_id, mq_subscribe, bulk_current_set, ble, blemsg
     try:

         currtime = getTime()
         print(f'currtime: {currtime}')
         topic = "masters/" + str(board_id)
         hs315_bulk = 0
         esp32Core = 0
         files = []
         files = os.listdir('log')
         try:
             if reg_14070[7] < 0:
                hs315_bulk = round((abs(reg_14070[7]) ^ 0xFFFF)/65536 * 120, 1) 
             else:
                hs315_bulk = round(reg_14070[7] /65536 * 120, 1)
             esp32Core = round((((raw_temperature() - 32) / 1.8) - 21), 1)
         except Exception as e:
                  print(f'*hs315_bulk read Error: {e}')
                  pass         
         message = {
                'TIME':currtime,
                'msgType' : "stack",
                "MASTER_ID": board_id,
                "KCELL1" : online[2],
                "KCELL2" : online[3],
                "KCELL3" : online[4],
                "KCELL4" : online[5],
                "maxCell1" : maxCell[2],
                "maxCell2" : maxCell[3],
                "maxCell3" : maxCell[4],
                "maxCell4" : maxCell[5],
                "Topv1" : topv[2],
                "Topv2" : topv[3],                
                "Topv3" : topv[4],
                "Topv4" : topv[5],                
                "topDiff" : topDiff,
                "core1" : core[2],
                "core2" : core[3],                
                "core3" : core[4],
                "core4" : core[5],
                "kCell_relay" : kCell_relay[2:6],
                "kCell_topv"  : kCell_topv[2:6],
                "kCell_reply" : kCell_reply[2:6],
                "kCell_down" : kCell_down,
                "bulk_current_set" : bulk_current_set,
                'bulk_current' :hs315_bulk,
                "stack_capacity": stack_capacity,
                "Circuit_breakers": Circuit_breakers,
                "esp32_core": esp32Core,
                "SOC": round(SOC,1),
                "save_files" : files,
                "Engine_battery":  Engine_battery,
                "Load_voltage":  Load_voltage,
                "Controller_mode":  Controller_mode,
                "Load_current":   Load_current,
                "Master_status": reg_800,
                "Loop"        : loop
     }
         payload = json.dumps(message)
         
         if ble and blemsg[0:5] == 'rs485':
             ble.write(f'online: {online[0:5]}')
             ble.write(f'online: {maxCell[0:5]}')
         if wlan.isconnected():
             print(f'publishing stack message')
             try:
                  wdt.feed()
                  print(f'publish: {client.publish(topic, payload)}')
                  wdt.feed()
                  #debug
#                   message['msgType'] = "stack-SAVD"
#                   payload = json.dumps(message)
#                   stringToWrite = str(payload) + "\r\n"
#                   WriteFile(stringToWrite)
                  
             except Exception as e:
                  print(f'*publishing stack Error: {e}')
                  pass
         else:
             try:
                  message['msgType'] = "stack-SAVD"
                  payload = json.dumps(message)
                  stringToWrite = str(payload) + "\r\n"
                  WriteFile(stringToWrite)
             except Exception as e:
                print(f'stack offline Error: {e}')
                pass                
         return True
     except Exception as e:
            print(f'publishing stack Error: {e}')
            pass
        
def sendmessage2(i):
     print(f'To HUM: {i}')
     global mq_topic, account_id, board_id, mq_subscribe
     try:
         now = time.ticks_ms()
         #message2 = "/triad/gen2/ecell"+str(i)+"/tx"
         #print(f'1 (seconds): {(time.ticks_ms() - now)/1000} ')
         curr_time = 'time.localtime()'
         #currtime = time.strftime("%Y-%m-%d %H:%M:%S",curr_time)
         currtime = getTime()
         print(f'currtime: {currtime}')
         #print(f'2 (seconds): {(time.ticks_ms() - now)/1000} ')
         #time.sleep(1.5)
         
         idlist = m.read_input_registers(i,2,6)
         print(f'i: {i} idlist: {idlist}')
         idint = ''.join(str(x) for x in idlist)
         topic = "masters/" + str(board_id)
         cycle1 ='{:010b}'.format(ecell_2[34])
         cycle2 ='{:010b}'.format(ecell_2[35])
         status1 = ecell_2[26] & ~(1 << 12)
         if wlan.isconnected():
             real_time = True
         else:
             real_time = False
             
         message = {
                'TIME':currtime,
                'msgType' : "kCell",
                "SLAVE_ID": idint,
                "MASTER_ID": board_id,
                "RS485_ID" : i,
                "CELL01":ecell_2[0]/1000,
                "CELL02":ecell_2[1]/1000,
                "CELL03":ecell_2[2]/1000,
                "CELL04":ecell_2[3]/1000,
                "CELL05":ecell_2[4]/1000,
                "CELL06":ecell_2[5]/1000,
                "CELL07":ecell_2[6]/1000,
                "CELL08":ecell_2[7]/1000,
                "CELL09":ecell_2[8]/1000,
                "CELL10":ecell_2[9]/1000,
                "CELL11":ecell_2[10]/1000,
                "CELL12":ecell_2[11]/1000,
                "CELL13":ecell_2[12]/1000,
                "CELL14":ecell_2[13]/1000,
                "CELL15":ecell_2[14]/1000,
                "CELL16":ecell_2[15]/1000,
                "CELL17":ecell_2[16]/1000,
                "CELL18":ecell_2[17]/1000,
                "CELL19":ecell_2[18]/1000,
                "CELL20":ecell_2[19]/1000,
                "topv":ecell_2[20]/100,
                "Sum":ecell_2[27]/100,     #ecell_2[27]/1000,
                "topv_sum_diff":ecell_2[28]/1000,
                "Avg":ecell_2[29]/1000,
                "cell_diff": round((max(ecell_2[0:20])/1000) - (min(ecell_2[0:20])/1000),3),
                
                
                "primary_relay":int(ecell_2[21]/1000),
                "secondary_relay":int(ecell_2[22]/1000),
                "Key_Sw_Sns":int(ecell_2[23]/1000),
                "core_temperature":ecell_2[24]/1000,
                "status":'{:016b}'.format(status1),
                "real_time": real_time,
                "revision": ecell_2[30]/1000,
                "aux1": int(ecell_2[31]/1000),
                "status2":'{:016b}'.format(ecell_2[32]),
                "Stdev": int(ecell_2[33]/1000),
                "cycle" : cycle1 + cycle2 
               }
         payload = json.dumps(message)
         
         if ble and blemsg[0:5] == 'rs485':
             if status1 == 0:
                 status1 = 'OK'
             print(f'sending ble: slave {i} status: {status1}')
             ble.write(f'slave {i} status: {status1}')
#          if ble and blemsg[0:5] == 'rs485':
#              ble.write(f'rs485')
#              blemsg = ""
             
         if wlan.isconnected() :
             print(f'publishing kCell message')
             try:
                  wdt.feed()
                  print(f'publish: {client.publish(topic, payload)}')
                  wdt.feed()
                  #debug

             except Exception as e:
                  print(f'*publishing kCell Error: {e}')
                  pass

         else:
             try:
                  if status1 > 0:
                      message['msgType'] = "kCell-SAVD"
                      payload = json.dumps(message)
                      stringToWrite = str(payload) + "\r\n"
                      WriteFile(stringToWrite)
             except Exception as e:
                print(f'kCell offline Error: {e}')
                pass
         return True
        
        
     except BaseException as err:
            sys.print_exception(err)        
        
        
#      except Exception as e:
#             exc_type, exc_obj, exc_tb = sys.exc_info()
#             print(f'publishing kCell Error: {e} - {exc_tb.tb_lineno}')
#             pass
    
def contains(str, set):
    for c in set:
        if c in str: return 1
    return 0

# def sendOffline():
#     idint = 'bugbug'
#     data = []
#     count = 0
#     try:
#         with open(LogFileName) as file_in:
#             for line in file_in:
#                 wdt.feed()
#                 count += 1
#                 if not line: 
#                     break
#                 data = json.loads(line)
#                 #print(f'CELL01... {data["CELL01"]}')
#                 idint = data["ID"]
#                 payload = json.dumps(data)
#                 message2 = mq_topic + str(account_id) +"/ecell/"+str(idint)
#                 client.publish(message2, payload)
#         print(f'Offline count: {count}')
#         #print(f'File size: {os.size("log.txt")}')
#         # removing
#         os.remove(LogFileName)
#     except  Exception as e:
#       print(f'sendOffline Error... {count} {e}')
#       pass
# if not wlan.isconnected():
#     print(f'connecting to network... {wifi_ssid}')
#     connectWiFi(wifi_ssid, wifi_pwd)
#     time.sleep(2)
#     restart_and_reconnect()
# else:
#     try:
#       client = connect_and_subscribe()
#     except OSError as e:
#       restart_and_reconnect()
#       pass
def bulk_mode(current):
    global bulk_current_set
    if bulk_current_set == current:
        return
    current_scale = int(current * 493) # scale 1/65536
    try:
      m.write_multiple_registers(1, 14077, [current_scale,0])
      bulk_current_set = current
      print(f'Set bulk_mode current to... {current} amps')
    except Exception as e:
      print(f'bulk_mode Error... {e}')
      pass

# Function to clear the kth bit of n
def clearBit(n, k):
    return (n & ( ~(1 << (k - 1))))    

msg = f'{getTime()} - BOOT'
#logFile.write(str(msg)+"\n")
# save the data using a buffer
#logFile.flush()

m = uModBusSerial(2, baudrate=9600, pins=[Pin(17), Pin(16,Pin.PULL_UP)], ctrl_pin=5)

char_timeout = 20000
start_discharge = utime.time()
loop = 0
start_Engine = 0
start_Engine_tmr = 0
fuel_per_minute =  0.47551/60 #1.8/60 
Engine_fuel_used = 5.00
prev_Engine_status = 2
Discharge_time = 0
Engine_run_time = 0
Last_Discharge_time = 0
start_discharge = utime.time()
start_Engine_tmr = 0
Batt_current = 0
Load_voltage = 0
Controller_mode = 0
Load_current = 0

slaveRst1 = 0
slaveRst2 = 0
slaveRst3 = 0
slaveRst4 = 0
slaveRst5 = 0
slaveRst6 = 0
function = 0
register = 0
count = 0
rtu_flag = 0
slaveID = 0
data = 0
cal_flag = 0

blepw = False
blemsg = ''
wifi_Tmr = utime.time()
time.sleep(5)
print(f'WIFI try 1 ')
try:
    if not wlan.isconnected():
            wdt.feed()
            restart_and_reconnect()
            print(f'IP: {wlan.ifconfig()[0]} ')
except Exception as e:
    print(f'WIFI try 1 error: {e}')
    
    pass

LogFileName = f"{utime.time()}.txt"
client_id = ubinascii.hexlify(machine.unique_id())
while True:   
    try:
        #break
        ## Read HS315 registers (1,register 13010 = 123)
        wdt.feed()
        gc.collect()
        print('Func definition: {} allocated: {}'.format(gc.mem_free(), gc.mem_alloc()))       

        loop = loop + 1
        print(f'Loop: {loop} {client_id} ')

        
##************LOG FILE***************
#         stringToWrite = "my test" + "\r\n" #debug
#         WriteFile(stringToWrite)
        try:
            list_of_files = os.listdir('log')
            if LogFileName in os.listdir('log'):
#                 print(f'CheckFileSize: {CheckFileSize()}  Max_File_Size: {Max_File_Size}')
                if CheckFileSize() > Max_File_Size:
                   LogFileName = f"{utime.time()}.txt"
                   print(f'new file {LogFileName}')
                   list_of_files = os.listdir('log')
                       
            if len(list_of_files) > 3:
                print(f'list_of_files: {list_of_files}')
                remove = f'/log/{list_of_files[0]}'
                print(f'remove: {remove}')
                os.remove(remove)
        except Exception as e:
            print(f'Log file error {e}')            
##***********************************

        if not wlan.isconnected():
#             print(f'Loop: {loop} wifi_Tmr: {utime.time()-wifi_Tmr} sec.')
            while i in range(1,4):
                LED_FileWrite.value(1)
                time.sleep(.250)
                LED_FileWrite.value(0)
            if (utime.time()-wifi_Tmr) > 300:
                wdt.feed()
                restart_and_reconnect()
                wifi_Tmr = utime.time()
        
        if wlan.isconnected():
            try:
               # if LogFileName in os.listdir(): sendOffline()
                wdt.feed()
                settime()
                wdt.feed()
                print(f'IP: {wlan.ifconfig()[0]} ')
                try:
                    print(f'check_msg: {client.check_msg()} ')
                    sendOffline()
                except Exception as e:
                    print(f'client error {e}')
                    if wlan.isconnected():
                        client = connect_and_subscribe()
                    pass
                wdt.feed()
                
                
                print(f'')
            except Exception as e:
                wdt.feed()
                print(f'check_msg error {e}')

                pass
            
        try:
            if RCS: 
               print(f':RCS-1  {m.read_coils(RCS[0],RCS[1],RCS[2])}  {RCS}')
               RCS = []
            if RIS:
               print(f':RIS-2  {m.read_discrete_inputs(RIS[0],RIS[1],RIS[2])}  {RIS}')
               RIS = []
            if RHR:
                
               sendRtuMsg(RHR) 
               msg = m.read_holding_registers(RHR[0],RHR[1],RHR[2])
               sendRtuMsg(msg)
               print(f':RHR-3  {msg}  {RHR}')
               RHR = []
            if RIR:
                  
               sendRtuMsg(RIR) 
               msg = m.read_input_registers(RIR[0],RIR[1],RIR[2])
               sendRtuMsg(msg)
               print(f':RIR-4  {msg}  {RIR}')                
               RIR = []
            if WSC:
               print(f':WSC-5  {m.write_single_coil(WSC[0],WSC[1],WSC[2]*65280)}  {WSC}')
               WSC = []
            if WSR:
               print(f':WSR-6  {m.write_single_register(WSR[0],WSR[1],WSR[2])}  {WSR}')
               WSR = []
            if WMC:
               print(f':FMC-15 {m.wite_multiple_coils(WMC[0],WMC[1],WMC[2])}  {WMC}')
               WMC = []
            if WMR:    
               print(f':WMR-16 {m.write_multiple_registers(WMR[0],WMR[1],WMR[2])}  {WMR}')
               WMR = []       
        except Exception as e:
            print(f'mqtt-rtu error {e}')
            RCS = []
            RIS = []
            RIR = []
            RHR = []
            WSC = []
            WSR = []
            WMC = []
            WMR = []
            pass



           
        reg_current   = [0 ,0]#m.read_input_registers(10,0,10)
        
        if ble:
           try: 
                if len(ble_msg) > 1:
                    print(f'ble_msg: {ble_msg}')
                    if ble_msg == 'Tri@d':
                        blepw = True
                        ble.write(f'password accepted')
                    if blepw:
                        ble.write(f'ble_msg: {ble_msg}')
                        blemsg = ble_msg
                        if len(ble_msg) > 4:
                            if blemsg[0:5] == 'ecell':
                                ble.write(f'please wait for data')                    
                        
                    ble_msg = ""
            

                    
                if blemsg[0:7] == 'bootsel':
                    ble.write(f'BOOTSEL')
                    msg = blemsg.split()
                    blemsg = ""                
                    if len(msg) > 1:
                        ble.write(f'BOOTSEL SLAVE {int(msg[1])}')
                        ble.write(f'{m.write_single_coil(int(msg[1]),1,0)}')
                    else:
                        ble.write(f'BOOTSEL MASTER')
                        #machine.bootloader()

#                 if blemsg[0:5] == 'rs485':
#                     msg = blemsg.split()
#                     blemsg = ""                
#                     if len(msg) > 1:
#                         ble.write(f'RESET SLAVE {int(msg[1])}')
#                         ble.write(f'{m.write_single_coil(int(msg[1]),0,0)}')
#                     else:
#                         ble.write(f'RESET MASTER')
#                         machine.reset()

#                 if blemsg[0:6] == 'reboot':
#                     msg = blemsg.split()
#                     blemsg = ""                
#                     if len(msg) > 1:
#                         ble.write(f'RESET SLAVE {int(msg[1])}')
#                         ble.write(f'{m.write_single_coil(int(msg[1]),0,0)}')
#                     else:
#                         ble.write(f'RESET MASTER')
#                         machine.reset()

#                 if blemsg[0:4] == 'scan':
#                     blemsg = ""
#                     for i in range(2, 14):
#                         wdt.feed()
#                         try:
#                             print(f'ID SCAN: ECELL {i}')
#                             idlist = m.read_input_registers(i,2,6)
#                             if len(idlist) > 0:
#                                 idint = ''.join(str(x) for x in idlist)
#                                 ble.write(f'ID-(ECELL {i}): {idint}')
#                         except Exception as e:
#                             print(f"ID {i} READ ERROR : {e} ")
#                             print('')
#                             pass                
                    
                if blemsg[0:6] == 'config':
                   blemsg = ""
                   wdt.feed()
                   try:                
                        with open("config.json") as json_file:
                            config = ujson.load(json_file)
                            print(f"IN: {config}")                 
#                         ble.write(f'mq_broker - { config["mq_broker"]}') 
#                         ble.write(f'mq_user - { config["mq_user"]}')
#                         ble.write(f'mq_pwd - { config["mq_pwd"]}')
#                         ble.write(f'mq_port - { config["mq_port"]}')
#                         ble.write(f'mq_topic - { config["mq_topic"]}')
#                         ble.write(f'mq_subscribe - { config["mq_subscribe"]}')
                        ble.write(f'wifi_ssid - { config["wifi_ssid"]}')
                        ble.write(f'wifi_pwd - { config["wifi_pwd"]}')
                        ble.write(f'wifi_en - { config["wifi_en"]}')
                        ble.write(f'"account_id" - { config["account_id"]}')
                        ble.write(f'"board_id" - { config["board_id"]}')
                         
                        ble.write(f'done reading config')
                        wdt.feed()
                        mq_broker = config["mq_broker"]
                        mq_user = config["mq_user"]
                        mq_pwd = config["mq_pwd"]
                        mq_port = config["mq_port"]
                        mq_topic = config["mq_topic"]
                        mq_subscribe = config["mq_subscribe"]
                        wifi_ssid = config["wifi_ssid"]
                        wifi_pwd = config["wifi_pwd"]
                        wifi_en = config["wifi_en"]
                        account_id = config["account_id"]
                        board_id = config["board_id"]
                        wdt.feed()
                   except Exception as e:
                        ble.write(f"ERROR: load_config {e}")
                        time.sleep(2)                
                        blemsg = ""
                        pass

                        
                if blemsg[0:9] == 'setconfig':
                     msg = blemsg.split()
                     blemsg = ""
                     if contains(':', msg[2]):
                         msg[2] = msg[2].replace(':', ' ')
                     wdt.feed()
                     try:
                        with open('config.json') as json_file:
                            data = ujson.load(json_file)
                        data[msg[1]] = msg[2]
                        with open('config.json', 'w') as outfile:
                            ujson.dump(data, outfile)
                        ble.write(f'done updating config')
                        wdt.feed()
                     except Exception as e:
                        ble.write(f'ERROR: updating {msg[1]} {e}')                
                        pass
           except Exception as e:
               blemsg = ""
               ble.write(f'ERROR: ble error {e}')
             #  print(f'ERROR: ble error {e}')                
               pass
            
        try:
            reg_1100 = m.read_input_registers(1,1100,10)
            Load_voltage = reg_1100[4]/256
            print(f"Load_voltage : {Load_voltage} ")
        except Exception as e:
              print(f"Load voltage READ ERROR : {e} ")
              pass

        if loop == 1:
           for i in range(2, 6):
               wdt.feed()
               now = time.ticks_ms() 
               try: 
                   m.write_single_coil(i,6,0)
               except Exception as e:
                   print(f"Scan {i} READ ERROR : {e} ")
                   pass
               print(f'2 (seconds): {(time.ticks_ms() - now)/1000} ')
           time.sleep(2)
            
          # topv = [Load_voltage,Load_voltage,Load_voltage,Load_voltage,Load_voltage,Load_voltage]

        
        
        for i in range(2, 6):
            wdt.feed()
            gc.collect()
            try:
                ecell_2  = m.read_input_registers(i,0,23)
                print(f'{ecell_2} ')
                online[i] = True
                kCell_reply[i] = 1
                if len(ecell_2) > 0:

                    status =   '{:016b}'.format(ecell_2[26])
                    print(f'status: {status}')
                    maxCell[i] = (max(ecell_2[0:20])/1000)
                    topv[i] = ecell_2[20]/100
                    core[i] = ecell_2[24]/1000
                    if ecell_2[26] > 0:
                        online[i] = 2
                    #if wlan.isconnected():
                        
                    # Check for down kCell    
                    kCell_relay[i] = int(ecell_2[21]/1000)
                    if abs(Load_voltage - topv[i]) <= .7:
                        kCell_topv[i] = 1
                    else:
                        kCell_topv[i] = 0
                                            
                        
                        
#                     if wlan.isconnected():    
                    print(f'MQTT: {sendmessage2(i)}')
                        #sendmessage3(i,ecell_2)
                       
            except Exception as e:
                print(f"ECELL {i} READ ERROR : {e} ")
                online[i] = 0
                maxCell[i] = 0
                topv[i] = 0
                core[i] = 0
                kCell_relay[i] = 0
                kCell_topv[i] = 0
                kCell_reply[i] = 0                
                #time.sleep(5)
                pass         
        
#         if len(ecell_2) > 10:
        
        topDiff = round( max(topv[2:6]) - min(topv[2:6]),3)
        
        NumList1 = kCell_relay[2:6]
        NumList2 = kCell_topv[2:6]
        NumList3 = kCell_topv[2:6]        
        total = []
        for j in range(4):
            total.append( NumList1[j] + NumList2[j] + NumList3[j])
        kCell_down = len([i for i in total if i < 3])     
        stack_capacity = (4 - kCell_down) * 30
        print(f" Online: {online[2:6]} maxCell: {maxCell[2:6]} Topv: {topv[2:6]} kCell_relay: {kCell_relay[2:6]} kCell_topv: {kCell_topv[2:6]} kCell_reply: {kCell_reply[2:6]}  topvDiff: {topDiff} core: {core[2:6]} stack_capacity: {stack_capacity} Circuit_breakers: {Circuit_breakers} SOC: {SOC}")
#         if wlan.isconnected():
        time.sleep(2)
        sendmessage4(online,maxCell,topv,topDiff,core,kCell_relay,kCell_topv,kCell_reply,kCell_down, stack_capacity, Circuit_breakers, SOC)

        wdt.feed()
        gc.collect()
#         print(f'debug 2')
        reg_70   = m.read_input_registers(1,70,10)
        reg_80   = m.read_input_registers(1,80,10)
        reg_120  = m.read_input_registers(1,120,10)
        wdt.feed()
        gc.collect()
        reg_130  = m.read_input_registers(1,130,10) #off by -1 
        reg_200 = m.read_input_registers(1,200,10)
        reg_210 = m.read_input_registers(1,210,10)
        reg_220 = m.read_input_registers(1,220,10)
        wdt.feed()
        reg_230 = m.read_input_registers(1,230,10)
        reg_279  = m.read_input_registers(1,279,20)
        reg_300  = m.read_input_registers(1,300,10)
        
        wdt.feed()
        gc.collect()
        reg_800 = m.read_input_registers(1,800,20)
        reg_1000 = m.read_input_registers(1,1000,10)
        reg_1010 = m.read_input_registers(1,1010,10)
        reg_1020 = m.read_input_registers(1,1020,10)
        reg_1030 = m.read_input_registers(1,1030,10)
        wdt.feed()
        gc.collect()
        reg_1040 = m.read_input_registers(1,1040,10)        
        reg_1070 = m.read_input_registers(1,1070,10)
        reg_1080 = m.read_input_registers(1,1080,10)
        reg_1090 = m.read_input_registers(1,1090,10)
        reg_1040 = m.read_input_registers(1,1040,10)
        wdt.feed()
        gc.collect()
        reg_1100 = m.read_input_registers(1,1100,10)
        reg_1110 = m.read_input_registers(1,1110,10)
        wdt.feed()
        reg_0 = m.read_input_registers(1,0,10)
        reg_13470 = m.read_input_registers(1,13470,10)
        reg_13240 = m.read_holding_registers(1,13240,10)
        reg_14200 = m.read_holding_registers(1,14200,10)
        reg_14070 = m.read_holding_registers(1,14070,10)
        hReg_70 = m.read_holding_registers(1,70,10)
        _Status_bar = m.read_input_registers(1,166,1)

        print(f'reg_800: {reg_800}')
        wdt.feed()
        gc.collect()
        AC_Current = reg_current[1]*.001
        AC_voltage = reg_current[0]*.1
        Gen_voltage = reg_1100[0]/256
        GCB_voltage = reg_1100[2]/256
        Load_voltage = reg_1100[4]/256
        Load_current = reg_1110[2]/256
        Engine_coolant = round((float(reg_70[7]/256) * 9/5) + 32,2) 
        Engine_oil = (reg_70[6]/256)*14.504
        Engine_rpm = reg_80[0]
        Engine_hours = (reg_70[1]*256)+reg_70[0]
        Engine_battery = reg_70[3]/256
        Engine_status = reg_130[1]
        Controller_mode = reg_120[0]
        Circuit_breakers = bin(reg_130[7])
        Battery_charge = reg_1040[0]
        Battery_status = reg_1040[1]
        Speed_governor_pct = (reg_1040[5]/256)
        Current_charge = reg_1020[2]
        SOC = reg_1030[3]/256
        End_discharge  = reg_1030[4]/256
        End_charge = reg_1030[6]/256        
        Batt_Neg = reg_1110[1]
        Charge_Cycles = reg_1000[0]
        Discharge_Cycles = reg_1000[2]
        Engine_fuel_used = reg_70[9]/256
        Battery_temperature = reg_1040[3]/256
        Status_bar = _Status_bar
#         print(f'Batt_Neg: {Batt_Neg}')
        if Batt_Neg >= 0:
            if Batt_Neg > 0:
                Batt_current = 256 - (reg_1110[0]/256)
                Gen_current = Batt_current + Load_current
            else:
                Gen_current = 0
#             print("*")
        else:                   
            Batt_current = abs(reg_1110[0])/256
            Gen_current = Batt_current + Load_current
#             print("**")

        Gen_watts = Gen_voltage * Gen_current
        Load_watts = Load_voltage * Load_current
#         print(f'Gen_current: {Gen_current}')
#         print(f'SOC: {SOC:.0f} %')
        print('****************************************************')
         

        #prev_Engine_status = 0 #debug mrp 042522

        #Engine_Start
        if Engine_status == 10:
            if prev_Engine_status != 10:
               for i in range(2, 6):
                   wdt.feed()
                   try: 
                       m.write_single_coil(i,6,0)
                   except Exception as e:
                       print(f"Scan {i} READ ERROR : {e} ")
                       pass
               
               if loop > 2: 
                  m.write_single_register(1, 78, 0) # Set SOC to zero
               start_Engine = utime.time()
               start_Engine_tmr = utime.time()
               end_discharge =  utime.time()
               Discharge_time = end_discharge-start_discharge
               Last_Discharge_time = Discharge_time
               bulk_bit =  [1,1,1,1]
            else:
               Engine_run_time  = utime.time() - start_Engine
               #Engine_fuel_used = Engine_fuel_used - (fuel_per_minute * ((utime.time() - start_Engine_tmr)/60))
               start_Engine_tmr = utime.time()
               if loop < 3:
                   bulk_bit =  [0,0,0,0]
               if stack_capacity > 0 and Engine_run_time < 211:    
                   if Engine_run_time < 120 and bulk_bit[0] :
                       bulk_bit[0] = 0
                       print(f'Engine_run_time: {Engine_run_time}')
                       bulk_mode(stack_capacity * .25)
                   if Engine_run_time > 120 and bulk_bit[1] :
                       bulk_bit[1] = 0
    #                    print(f'Engine_run_time: {Engine_run_time}')
                       bulk_mode(stack_capacity * .5)
                   if Engine_run_time > 180 and bulk_bit[2] :
                       bulk_bit[2] = 0
    #                    print(f'Engine_run_time: {Engine_run_time}')
                       bulk_mode(stack_capacity * .75)
                   if Engine_run_time > 210 and bulk_bit[3] :
                       bulk_bit[3] = 0
    #                    print(f'Engine_run_time: {Engine_run_time}')
                       bulk_mode(stack_capacity)
               else:
                       if stack_capacity > 0:
                           bulk_mode(stack_capacity)                       
                       else:
                           bulk_mode(30)
        else:
           #Engine_Stopped 
           if Engine_status == 2:
               Discharge_time = utime.time() - start_discharge #Debug 050222
               if prev_Engine_status != 2:
                   bulk_current_set = 0
                   start_discharge = utime.time()
                   end_Engine = utime.time()
                   Engine_run_time  = end_Engine-start_Engine
                   #Engine_fuel_used = Engine_fuel_used - (fuel_per_minute * (Engine_run_time/60))
                   pass
         
        prev_Engine_status = Engine_status
        #Send Mqtt
        #sendmessage1()
        if Engine_status == 10:
            update_rate = 1
        else:
            update_rate = 1
        print(f" update_rate: {update_rate}")    
        time.sleep(update_rate)
        #wdt.feed()
        try:
#             if wlan.isconnected():
                
            sendmessage1()
                
        except Exception as e:
            print(f"Client NOT CONNECTED: {e}")
            pass        
        
     
        try:
             if ble and blemsg == 'hs315':
                ble.write(f'Time: {getTime()}')  
                ble.write(f'Gen_voltage: {Gen_voltage:.2f} V')
                ble.write(f'Gen_watts: {Gen_watts:.0f} W')
                ble.write(f'Load_voltage: {Load_voltage:.2f} V')
                ble.write(f'Load_watts: {Load_watts:.0f} W')
                ble.write(f'SOC: {SOC:.0f} %')
                ble.write(f'Engine_battery: {Engine_battery:.1f} V')
                ble.write(f'Engine_rpm: {Engine_rpm} RPM')
                ble.write(f'Engine_hours: {Engine_hours} Hrs')
                ble.write(f'Engine_oil: {Engine_oil:.0f} PSI')
                ble.write(f'Engine_coolant: {Engine_coolant:.0f} C')
                ble.write(f'Engine_run_time: {Engine_run_time} Sec')
                ble.write(f'Engine_fuel_used: {Engine_fuel_used:.4f} Gal')
                ble.write(f'')
                print(f"BLE SENT:")
        except Exception as e:
            print(f"BLE ERROR: {e}")
            pass
        
    except Exception as e:
        print(f" HS315 ERROR : {e}")
        wdt.feed()
        pass
  #     except KeyboardInterrupt:
#         pass
