import pandas as pd
from PIL import Image

RESULT_NAME = "Result.xlsx"

dataframe1 = pd.read_excel(RESULT_NAME)

print(dataframe1)

im = Image.new(mode="RGB", size=(200,200), color='red')

im.save("resultingImg.png")

