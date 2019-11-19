#Read sensor value
dis = BP.get_sensor(BP.PORT_1)

#Use sensor reading to print line
#Origin at 500, 500
origin = 500
#Fetch rotation angle
angle = BP.get motor encoder(BP.PORT A)

#Print lines
print ("drawLine:" + str(origin,origin, origin + dis * math.cos(angle), origin + math.sin(angle))
