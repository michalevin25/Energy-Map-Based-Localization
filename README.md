# Energy-Map-Based-Localization

## Explanation about the algorithm:
My approach was to create a simple and as straight-forward as possible algorithm. 
### The steps were as following:
1. The odometry trajectory was rotated, I'm assuming since it was recorded in a body frame. I corrected it to the compass (and GNSS) global frame.
2. I fused the odometry and compass, using the position from the odometry and the heading from the compass. Here I took into caclulation the sensors' errors, and since the odometry has an inherent drift, I trusted the compass more.
4. Map matching: This part was done in a couple of steps, going from coarse to fine. The reason for this was processing time, and to increase accuracy.
   3a. In each step there was a matching energy function: The energy in the estimated trajectory is compared to the map's energy values until the best aligned energy is found.
   3b. Initially, The estimated trajectory was out of map's bounds. Therefore, it was pushed to be inside the bounds so that the algorithm wont search in non-relevant space.
   <img width="395" height="426" alt="image" src="https://github.com/user-attachments/assets/6a9760d6-6d60-44ae-9fbd-ad02cfc5a439" />

   result of pushing to boundaries:
   
   <img width="450" height="426" alt="image" src="https://github.com/user-attachments/assets/c9c87bcf-2033-44c7-a62b-34de82d37612" />

   3c. The next goal was to find the starting point of the trajectory. In order to do so, I ran the matching energy function in three levels, from coarse to fine, in each level making the shifting variables finer and narrowing the   search area. Each was shift based on a trust factor alpha that balanced odometry drift and energy noise.

   Below is how the trajectory looked after aligning the starting point, but before local alignment (next step):
   
   <img width="450" height="409" alt="image" src="https://github.com/user-attachments/assets/a549ef60-1c06-4304-bea4-4fbc390886cd" />

   3d. After finding the initial starting point, a function that refines local alignment was applied. It splits the aligned trajectory into overlapping windows. If a window’s shift is too different from the previous one, it smooths the correction and counts it as an outlier. It then stitches the corrected windows together by averaging overlapping sections, returning a continuous aligned path with an outlier percentage.

### Overview of the alogirhm is below:

<img width="1117" height="270" alt="image" src="https://github.com/user-attachments/assets/9de74d2d-e508-4e46-9f82-349f3c671b1a" />

### Final result is below: 

<img width="312" height="319" alt="image" src="https://github.com/user-attachments/assets/1b1a00f9-eb8e-4eb4-a549-d72ac1f1a847" />


## Accuracy Metrics

- RMS Error: 37.824 m
- Max Position Error: 140.96
- 95th Percentile error: 123.98
- MAE: 18.33

Stability Analysis:
- number of outlier windows in window matching: 6.13 %

- Stability Analysis Plots
These plots illustrate the variation and distribution of the estimated map shifts across successive trajectory windows during the matching process:

<img width="1072" height="568" alt="image" src="https://github.com/user-attachments/assets/6f211d99-c87d-46b3-af0c-a904173698e2" />

Shift Variation Plots: These plots show the estimated shifts in X,Y. 
Shift Distribution Histogram – A 2D histogram showing the frequency of different shift values .

<img width="413" height="280" alt="image" src="https://github.com/user-attachments/assets/b7be9ae7-1e5e-4865-a24e-25263194aaf6" />

## Visualization

 **Energy map** with GNSS ground truth trajectory overlaid:
 
<img width="312" height="319" alt="image" src="https://github.com/user-attachments/assets/5fb2a1b2-452d-4fb2-8cd2-f15737062444" />

**Estimated vs. GNSS ground truth trajectory** comparison

<img width="312" height="319" alt="image" src="https://github.com/user-attachments/assets/1b1a00f9-eb8e-4eb4-a549-d72ac1f1a847" />

**Position error over time** (distance between estimated and GNSS positions)

<img width="389" height="278" alt="image" src="https://github.com/user-attachments/assets/e5e51f94-5fcf-4c67-9e16-345b0fffef72" />

**Error distribution histogram** to show error characteristics

<img width="568" height="352" alt="image" src="https://github.com/user-attachments/assets/a94a98b9-ca69-4fd5-97a1-bfb630296688" />



