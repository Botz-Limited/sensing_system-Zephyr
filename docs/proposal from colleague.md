Created 28.07.2028 (YA)

Based on the  “Form Segments” document (linked below), the proposed metrics making up the Form score have been categorised into what is feasible to determine now (based on the shoe data output) and what requires a bigger lift/more research. A deeper dive into the calculation and/or presentation of each metric is then listed.

[Form Segments (Concept Work).pdf](attachment:6c9993ee-2eda-4703-8a3b-ea17f1f75031:Form_Segments_(Concept_Work).pdf)

[Display of Metrics](https://www.notion.so/Display-of-Metrics-246c306db18b809d915be286ace94d5c?pvs=21)

### Categories:

1. Feasible to develop this metric now 
2. This metric can be measured/determined but more research is needed to understand how to correlate and validate the output shoe against other systems: (A) This can be done against the pressure plate or Vicon OR (B) We don’t have the appropriate system to validate it 
3. We can’t define this metric directly from the shoe and/or advanced algorithms are required. Raw data from the shoe are necessary for the development of this metric.

| **Form segment/ Category** | **Cadence** | **Symmetry** | **Strike** | **Base** | **Force** | **Stability/ Pronation** | **Alignment/ hip drop** | **Momentum/ lean/ axis** |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **1** | Cadence | Stride duration symmetry |  | Ground contact time |  |  |  |  |
|  |  | Stride length symmetry |  | Flight time |  |  |  |  |
|  |  | Ground contact time symmetry |  |  |  |  |  |  |
| **2A** |  |  | Foot strike (fore/mid/heel) |  |  | CPEI |  |  |
|  |  |  | Toe off angle |  |  | Pronation/ supination |  |  |
|  |  |  | Initial contact angle |  |  |  |  |  |
| **2B** |  |  |  |  | Impact force |  |  |  |
|  |  |  |  |  | Push off force |  |  |  |
|  |  |  |  |  | Peak forces  |  |  |  |
| **3** |  |  | Overstriding | Step width |  |  | Pelvic drop | Torso lean |
|  |  |  |  |  |  |  | Knee window | Vertical oscillation |

# Category 1:

The basis for this group of metrics is the ability to detect “Foot strike” and “Toe off events”. In Vicon this done using a kinematic based method as described in the paper by [Patoz et al. (2021)](https://www.sciencedirect.com/science/article/pii/S0021929021005042), while in the shoe this is based on readings from the pressure insoles. Besides stride length (and symmetry), these are all temporal measures. We can compare directly between different systems (Vicon, Stryd and the Botz shoe).

- Cadence
- Ground contact time (or stance time)
    - Ground contact symmetry
- Flight time (or swing time)
- *Stride duration (not included in original list)*
    - Stride duration symmetry
- *Stride length (not included in original list)*
    - Stride length symmetry

## Cadence

- Defined as the number of foot strikes within a certain time period
- How it is generally reported: as an aggregate of left and right foot strikes
- This is can be calculated as an instantaneous quantity (cadence based on the time between two consecutive foot strikes: cadence = 1/t), or as a moving average (the number of foot strike events per time period)
    - We need to decide which method is best for presenting data and if going with a moving average, how big that window is
- Foot strike events will be measured independently for each foot, but then will need to be aggregated to report the metric in a form that is consistent with other systems
    - For the shoe there could be some complexity in aggregating foot strikes from the left and right feet, and there are a couple of different methods that can be used for this

***Questions for Laura:***

1. Besides an average cadence for a certain running duration, how will this be presented in the app as a deeper dive to users? (Using graphs from other apps as a reference, Stryd plots a cadence value every 1s, while Garmin seems to plot 7 values in a 30s period)

## Ground Contact Time

- Defined as the duration of time from foot strike of one foot, to the subsequent toe-off of the same foot
- Again, this can be calculated as in instantaneous quantity (per stride)
- This is also measured independently for both feet
- If calculating an average GCT, this can be done per gait cycle (foot strike of one foot, to foot strike of the same foot - includes foot strike/toe-off of the other foot), or within a defined time period

***Questions for Laura:***

1. What do we want to present: GCT average value for both feet (this is what Stryd and Garmin do) or separate values for each foot?
2. Following on from the above, to we just want to present the GCT average for the run, or go into more detail and how fine a detail do we want this to be?

## Ground Contact Symmetry

- This is generally presented as a balance between left and right, so a user would get get two values e.g. 49.4% L/ 50.6% R
- If using 100% to describe perfect symmetry, then it’s important to define how you would describe asymmetry. The more asymmetry there is, the bigger the variation in calculating this with different methods. So it’s important to understand the reasoning behind how this is defined
- Due to internal clock drift, this is currently harder to calculate for the Botz shoe (as data from the left and right shoes aren’t necessarily aligned)

***Questions for Laura:***

1. Do you want to present a balance score as other apps do, or present GCT symmetry as a value out of 100%?
2. If it is the latter, how is this being defined?
3. As the previous metrics, how much of a breakdown do you want the users to see?

## Flight time (or swing time)

- Flight time is used in running and is measured as the duration of time where both feet are off the ground
    - This would be calculated as the time from toe-off of one foot to the foot strike of the opposite foot
- Swing time is used in walking and is measured as the duration of time where one foot is off the ground
    - This would be calculated as the time from toe-off, to the foot strike of the same foot
- If we calculate the flight times at different speeds (there may be minimal flight time in some walking scenarios), we can the determine a threshold to identify whether someone is walking or running, and whether the user should then be presented with the flight or swing time metric

***Questions for Laura:***

1. Do you want to present this as a standalone metric and if so, how? (average and/or more of a breakdown) 

## Stride duration (not included in original list)

- Defined as duration of time from foot strike of one foot to the next foot strike on the same foot
- The intermediate step required in order to get to stride duration symmetry
- An instantaneous metric (stride duration for every gait cycle)

## Stride duration symmetry

- No reference to use for presenting this metric, as it is not one used by Stryd or Garmin
- Would be calculated based off stride duration (maybe in a similar way to GCT symmetry)

***Questions for Laura:***

1. Do you want to present a balance score, or present stride duration symmetry as a value out of 100%?
2. If it is the latter, how is this being defined?
3. As the previous metrics, how much of a breakdown do you want the users to see?

## Stride length (not included in original list)

- Defined as the distance from toe-off of one foot, to foot strike of the same foot
    - in Vicon this would be calculated using the distance moved by the HEEL marker in the y-direction, or using foot contact at a fixed point and looking at marker trajectories (as we are on a moving surface). The approach used can be compared to an instrumented treadmill for accuracy
- A metric presented by both Stryd and Garmin (averaged between left and right feet to give one value) as an overall average and then a breakdown with more detail over the running period

***Questions for Laura:***

1. Do you want to present this as a standalone metric and if so, how? (average and/or more of a breakdown) 

## Stride length symmetry

- No reference to use for presenting this metric, as it is not one used by Stryd or Garmin (but stride length is)
- Would be calculated based off stride length (maybe in a similar way to GCT symmetry)

***Questions for Laura:***

1. Do you want to present this as a standalone metric and if so, how? (average and/or more of a breakdown) 
2. Do you want to present a balance score, or present stride length symmetry as a value out of 100%?
3. If it is the latter, how is this being defined?

# Category 2A

These are metrics we can calculate using the output of the insole sensor or the IMU in the shoe, however we need to better understand how some of these metrics are defined/how we want to define them and the appropriate values to use from the pressure plate and Vicon for comparison. If using the insole sensor to define these metrics, then the pressure plate would be a better comparison, while IMU-based metrics (using roll angles) would be better compared to Vicon. 

- Foot pronation/supinaiton/CPEI during step
    - can be defined in different ways, such as looking at ankle eversion, so we need to decide the most appropriate method for us
- Toe off angle
- Initial contact angle
- Foot strike (fore/mid/heel)

# Category 2B

These are metrics we can calculate using the insole sensor, but we do not have an in-house method of validating them. Access to an instrumented treadmill would be required.

- Force (push off)
- Force (impact)
- Force (peak)

# Category 3

These metrics cannot be defined directly from the shoe output and/or need advanced algorithms. These can be determined by interpreting the data from Vicon, so collecting the raw data from the shoe would put us in a position to infer these metrics from what we see in the Vicon data. 

- Step width
- Overstriding
- Torso lean
- Vertical oscillation
- Pelvic drop
- Knee window