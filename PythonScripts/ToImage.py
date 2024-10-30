import pandas as pd
from PIL import Image
import numpy as np
import math 

RESULT_NAME = "Data/Result.xlsx"

dataFrame = pd.read_excel(RESULT_NAME)

#removing Unneeded Variables 


#format to desired datatype and labels
readings = pd.DataFrame(dataFrame[1:])
readings = readings.rename(columns={'Var1':'Magnitude', 'Var2':'Elevation', 'Var3':'Azimuth'})

ConvertDict = {
    "Magnitude": int,
    "Elevation": int,
    "Azimuth": int
}

readings = readings.astype(ConvertDict)

#sort readings into desired reading order
readings = readings.sort_values(by=['Elevation','Azimuth'], axis=0, ignore_index=True)


print(readings)

#Calculate mean and standard dev
means = np.mean(readings, axis=0)
standardDev = np.std(readings, axis=0)

MagMean = means.iloc[0]
MagStd = standardDev.iloc[0]

print(means)
print(standardDev)

zScores = []

maxZ = 0
minZ = 100

#calculate the z- score of each reading
for x in readings["Magnitude"]:
    z = (x - MagMean)/MagStd
    zScores.append(float(z))

    if z > maxZ: 
        maxZ = z
    if z < minZ: 
        minZ = z 

print(minZ)
print(maxZ)


# determine the scale change and the offset 
highestWithOffset = maxZ - minZ

k = 255/highestWithOffset

print("Highest With Offset: " + str(highestWithOffset))
print("k: " + str(k))

# Create Image, defaulting to 180 for now
resultingImage = Image.new(mode="RGB", size=(180,180), color='black')


azimuth = readings['Azimuth'].astype(int)
elevation = readings['Elevation'].astype(int)

print(azimuth[1])

#determine the brightness for each z score and map to position on the image
for x in range(len(zScores)):
    brightness = k * (zScores[x]+minZ)

    if math.isnan(brightness):
        resultingImage.putpixel((azimuth[x],elevation[x]),200)
    else:
        resultingImage.putpixel((azimuth[x],elevation[x]),int(brightness))




#Save Image
resultingImage.save("Data/Image.png")

