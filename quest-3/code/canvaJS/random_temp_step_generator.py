# -*- coding: utf-8 -*-
"""
Created on Fri Oct  6 18:11:40 2023

@author: JimmyMan
"""

import csv
import time
import random

# Paths to the CSV files
csv_file0 = 'carmin0.csv'
csv_file1 = 'carmin1.csv'

# List of CSV files
csv_files = [csv_file0, csv_file1]

# Initialize or overwrite the CSV files with headers
for csv_file in csv_files:
    with open(csv_file, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Time", "Step", "Temp"])

while True:
    # Generate random step count between 0 and 10 and temperature between 15 and 35
    step = random.randint(0, 10)
    temp = random.uniform(15, 35)

    # Get the current time in HH:mm:ss format
    current_time = time.strftime('%H:%M:%S')

    # Choose a random CSV file from the list
    chosen_csv_file = random.choice(csv_files)

    # Append to the chosen CSV file
    with open(chosen_csv_file, 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([current_time, step, round(temp, 2)])

    # Wait for 1 second
    time.sleep(1)
