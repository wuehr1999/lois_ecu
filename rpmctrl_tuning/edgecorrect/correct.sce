clear()
clc()

raw = csvRead("rpmlog.csv", [], [], 'double') // Read csv

smallLeft = 0
smallRight = 0
counter = 0

// Calculate Factors by averaging
for m = 2:size(raw)(1)
    t1Left = raw(m - 1,  1)
    t2Left = raw(m, 1)
    if(t1Left > t2Left)
        tmp = t2Left
        t2Left = t1Left
        t1Left = tmp
    end
    smallLeft = smallLeft + t1Left / (t1Left + t2Left)
    
    t1Right = raw(m - 1,  2)
    t2Right = raw(m, 2)
    if(t1Right > t2Right)
        tmp = t2Right
        t2Right = t1Right
        t1Right = tmp
    end
    smallRight = smallRight + t1Right / (t1Right + t2Right)
    
    counter = counter + 1
end

deltaLeft = smallLeft / counter
deltaRight = smallRight / counter
corrfacLeftShort = 0.5 / deltaLeft
corrfacLeftLong = 0.5/ (1 - deltaLeft)
corrfacRightShort = 0.5 / deltaRight
corrfacRightLong = 0.5/ (1 - deltaRight)
disp(corrfacLeftShort)
disp(corrfacLeftLong)
disp(corrfacRightShort)
disp(corrfacRightLong)
