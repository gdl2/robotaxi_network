import pandas as pd
import h3
import numpy as np
from time import time

df = pd.read_csv("Transportation_Network_Providers_Trips.csv")

# Keep only rows that contain both pickup and dropoff locations
df = df.dropna(subset=['Pickup Centroid Location', 'Dropoff Centroid Location'])
print(df.dtypes)

# Group rows by H3 index
h3_resolution = 4

df['TSP Group'] = -1
group_num = 1
while -1 in df['TSP Group'].values:
    # numpy vectorize offers a magnitude faster performance than pandas apply()
    df['Pickup H3 Cell'] = np.vectorize(h3.geo_to_h3)(df['Pickup Centroid Latitude'], df['Pickup Centroid Longitude'], h3_resolution)
    df['Dropoff H3 Cell'] = np.vectorize(h3.geo_to_h3)(df['Dropoff Centroid Latitude'], df['Dropoff Centroid Longitude'], h3_resolution)

    for index, rows in df.groupby(by=['Pickup H3 Cell']):
        # Filter out groups that are "small enough" to be solved quickly using TSP
        if len(rows) <= 50 and -1 in rows['TSP Group'].values:
            df.loc[(df['Pickup H3 Cell'] == index) & (df['TSP Group'] == -1), 'TSP Group'] = group_num
            group_num += 1
        print(h3_resolution, index, len(rows))
    h3_resolution += 1

print(min(df['TSP Group']), max(df['TSP Group']), "groups")
print(df)


import requests
def getDurationMatrix(lnglats):
    url = "http://localhost:5000/table/v1/driving/"
    locations = ""
    for lng, lat in lnglats:
        locations += "{},{};".format(lng, lat)
    # Drop last ;
    locations = locations[:-1]
    r = requests.get(url+locations)
    res = r.json()
    try:
        duration_matrix = res['durations']
        # Add dummy depot by setting first row and first column to 0
        duration_matrix.insert(0, [0]*len(lnglats))
        for i in range(len(lnglats)+1):
            duration_matrix[i].insert(0, 0)
        return np.around(duration_matrix).astype(int).tolist()
    except:
        raise Exception("Table too big")

def getPickupDropoffPairs(pickup_dropoff_pairs_raw, lnglats_for_duration_matrix):
    dict_lnglatToIndex = {}
    for i in range(len(lnglats_for_duration_matrix)):
        dict_lnglatToIndex[lnglats_for_duration_matrix[i]] = i+1 # Add 1 to account for dummy variable
    pickup_dropoff_pairs = []
    for pickup_raw, dropoff_raw in pickup_dropoff_pairs_raw:
        pickup = dict_lnglatToIndex[pickup_raw]
        dropoff = dict_lnglatToIndex[dropoff_raw]
        pickup_dropoff_pairs.append([pickup, dropoff])
    return pickup_dropoff_pairs

from or_tools_helper import vrp_solver
start = time()
total_num_vehicles = 0

for group, rows in df.groupby(by=['TSP Group']):
    # Get pairs of pickup/dropoff locations
    pickups = zip(rows['Pickup Centroid Longitude'], rows['Pickup Centroid Latitude'])
    dropoffs = zip(rows['Dropoff Centroid Longitude'], rows['Dropoff Centroid Latitude'])

    # Remove trips that have same pickup and dropoff location
    pickup_dropoff_pairs_latlngs = list(zip(pickups, dropoffs))
    pickup_dropoff_pairs_indices = []
    lnglats_for_duration_matrix = []
    i = 0
    for pickup, dropoff in pickup_dropoff_pairs_latlngs:
        if pickup != dropoff:
            pickup_dropoff_pairs_indices.append([i*2+1, i*2+2])
            lnglats_for_duration_matrix.extend([pickup, dropoff])
            i += 1

    print(pickup_dropoff_pairs_indices)

    # Calculate the duration matrix
    duration_matrix = getDurationMatrix(lnglats_for_duration_matrix)

    #vehicle_start_locations = [0]*len(lnglats) # FIX THIS
    demands = [0]+[0,1]*i
    solution = vrp_solver(duration_matrix, pickup_dropoff_pairs_indices, demands)
    print(solution)
    total_num_vehicles += len(solution)

print(total_num_vehicles)
print(time() - start, "seconds")
