#!/usr/bin/env python3
import numpy as nps
from bagpy import bagreader
import pandas as pd

def main():

    data_file = '/home/moritz/Documents/Advanced_Robot_Learning/Manipulation/2024_ARL_demos_all/_2024-03-14-14-16-55.bag'

    # Load data
    data = load_data(data_file)

    # Process and segment the data
    processed_data = process_data(data)

    # Save the segmented data
    # save_data(processed_data, save_path)

def load_data(file):
    return bagreader(file)

def process_data(data):
    topics = data.topic_table
    print(topics)
    pass

def save_data(data, save_path):
    # Implement saving the processed data
    pass

if __name__ == '__main__':
    main()