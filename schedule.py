import json
from datetime import datetime
from lib.inout.inout import gp

today = []
fStart = False
fEnd = False
fSchedule = False

for i in range(24):
  today.append([0,0,0,0,0,0,0,0,0,0,0,0])

def readSchedule():
    schedule = {}
    
    with open("/home/pi/dambee_lh/schedule.json", "r") as sfile:
      try:
        schedule = json.load(sfile)
        sfile.close()
        fSchedule = True
        
      except:
        print("No Schedule Data")
        fSchedule = False
    
    print(schedule)
    
    if fSchedule == True:
        schedule = schedule['schedule']
        
        now = datetime.now()
        date = int(now.strftime('%Y%m%d'))
        week = now.weekday()
        if week == 6: week = 0
        else: week += 1
        
        print("Today : ", date, ", week : ", week)
        
        for sc in schedule:
            schedule = sc
            print(schedule)
            
            if schedule['startDt'] == None:
              start_date = date
              end_date = date + 10000000
            else:
              start_date = int(schedule['startDt'])
              end_date = int(schedule['endDt'])
            
            week_date = schedule['dayOfWeek']
            
            if week_date is not '':
              if len(week_date) > 1:
                week_date = week_date.split(',')
              dayofweek = [0,0,0,0,0,0,0]
              for i in week_date:
                dayofweek[int(i)] = 1
            else:
              dayofweek = [1,1,1,1,1,1,1]
                
            print(start_date, end_date, dayofweek)
                 
            if date >= start_date and date <= end_date and dayofweek[week] == 1 :
                start_hour = schedule['startTime'][0:2]
                start_min = schedule['startTime'][3:5]
                print(start_hour, start_min)
                
                end_hour = schedule['endTime'][0:2]
                end_min = schedule['endTime'][3:5]
                print(end_hour, end_min)
                
                sh = int(start_hour)
                sm = int(int(start_min) / 5)
                eh = int(end_hour)
                em = int(int(end_min) / 5)
                
                today[sh][sm] = 'S'
                today[eh][em] = 'E'

        print(today)

def checkSchedule():  
    #print(today)
    global fStart, fEnd
    
    now = datetime.now()
    cur_hour = int(now.strftime('%H'))
    cur_min = int(int(now.strftime('%M')) / 5)
    
    if today[cur_hour][cur_min] == 'S':
       fStart = True
       today[cur_hour][cur_min] = 0
       print("Schedule Start : ", cur_hour, cur_min * 5)
    elif today[cur_hour][cur_min] == 'E':
       fEnd = True
       today[cur_hour][cur_min] = 0
       print("Schedule End : ", cur_hour, cur_min * 5)
        
if __name__ == '__main__':
    readSchedule()
    checkSchedule()
