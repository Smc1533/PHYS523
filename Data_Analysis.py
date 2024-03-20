import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


data = pd.read_csv("data/TEST.csv")

def clean_number(number):
    # Split the number by the decimal point
    parts = number.split('.')
    
    # Remove everything after the first '-' sign if present
    if len(parts) > 1:
        if '-' in parts[1]:
            parts[1] = parts[1].split('-')[0]
    
    # Join the parts back together
        cleaned_number = '.'.join(parts[:2])
    
        return float(cleaned_number)
    else:
        return float(parts[0])

data["Z"] = data["Z"].apply(clean_number)
data["gZ"] = data["gZ"].apply(clean_number)
data['t'] = data['t'].div(1000)

for col in data:
    if col != 't':
        data.plot(x='t', y=col,
        kind="line", xlabel = ('Time (s)'), figsize=(10, 10))
        plt.show()

plt.scatter(data['X'], data['Y'], color='blue')

# Add circles around the points to represent errors
for _, row in data.iterrows():
    circle = plt.Circle((row['X'], row['Y']), row['eX'], color='red', fill=False)
    plt.gca().add_artist(circle)

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Data with Error Circles')
plt.grid(True)
plt.axis('equal') 
plt.show()