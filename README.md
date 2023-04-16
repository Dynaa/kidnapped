# Kidnapped Vehicle Project
Self-Driving Car Engineer Nanodegree Program

In this project we will utilize a particle filter. Particle filters are the realisation of Bayes filters or Markov localisation filter. Particle filters concepts are mainly used to solve the localisation problems. 


<p align="center">
  <img src="https://miro.medium.com/v2/resize:fit:1400/format:webp/1*9Ynn4FfWynRo3rnhj7ic5g.png" width="350" title="Particle Filter illustration from Udacity lecture">
</p>


As you can see in the above picture, red dots are the discrete guesses of where the robot might be. Each red dot has x coordinates, y coordinates, and orientation. Particle filter is the set of several thousand such guesses comprise approximate representation of the posterior of the robot. In the beginning, particles are uniformly spread, but filter make them survive in proportion to how consistent is the particles with sensor measurements.

## Weight

<p align="center">
    <img src="https://miro.medium.com/v2/resize:fit:1400/1*A__oGj_OtdgywyPrIc3Ppw.gif">
</p>

Particle filter carry discrete number of particles. Each particle is a vector which contains x coordinates, y coordinates, and orientation. Particles survive based on how they are consistent with sensor measurements. The consistency is measured based on mismatch between actual measurement and predicted measurement which is called weights.

The weight implies how close the actual measurement of the particle to predicted measurement. In particle filter, higher weight particles has got higher probability to survive. In other words, probability of survival of each particle is proportional to weights.

## Resampling
Resampling is the technique used to randomly drawing new particles from old ones with replacement in proportion to their importance weights. After resampling, particles with higher weights likely to stay and all others may die out.

In order to do the resampling, Resampling Wheel technique is used.

## Particle Filters implementation
![image](https://user-images.githubusercontent.com/677133/232330557-b5e08c4e-62b4-4b4f-847b-6a6f4e555147.png)

Particle filters has got four major steps:

- Initialisation step: At the initialization step we estimate our position from GPS input. The subsequent steps in the process will refine this estimate to localize our vehicle.
- Prediction step: During the prediction step we add the control input (yaw rate & velocity) for all particles.
- Update step: During the update step, we update our particle weights using map landmark positions and feature measurements.
- Resample step: During resampling we will resample M times (M is range of 0 to length_of_particleArray) drawing a particle i (i is the particle index) proportional to its weight . Resampling wheel is used at this step.

### 1. Initialisation step
Very first thing in Particle filter is to initialise all the particles. At this step we have to decide how many particles we want to use. Generally we have to come up with a good number as it wont be too small that will prone to error or too high so that it is computationally expensive. General approach to initialisation step is to use GPS input to estimate our position.

And final code for the initialisation step:

    if(is_initialized) {
	    return;
    } 

    //Number of particles
    num_particles = 100;

    //SD
    double std_x = std[0];
    double std_y = std[1];
    double std_theta = std[2];

    //Normal distributions
    normal_distribution<double> dist_x(x, std_x);
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> dist_theta(theta, std_theta);

    //Generate particles with normal distribution with mean on GPS values.
    for(int i = 0; i < num_particles; i++) {
        Particle p;
        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1.0;
        particles.push_back(p);
    }
    is_initialized = true;


### 2. Prediction step
Now that we have initialized our particles it’s time to predict the vehicle’s position. Here we will use below formula to predict where the vehicle will be at the next time step, by updating based on yaw rate and velocity, while accounting for Gaussian sensor noise.

<img src="https://user-images.githubusercontent.com/677133/232331648-b7797713-39f4-4648-a59a-2262cdb6370c.png" width="350">


The equations for updating x, y and the yaw angle when the yaw rate is not equal to zero
And final code for the prediction step:

    //Normal distributions for sensor noise
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);

    for(int i = 0; i < num_particles; i++) {
        if(fabs(yaw_rate) < 0.00001) {  
            particles[i].x += velocity * delta_t * cos(particles[i].theta);
            particles[i].y += velocity * delta_t * sin(particles[i].theta);
        } 
        else{
            particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
            particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
            particles[i].theta += yaw_rate * delta_t;
        }

        //Noise
        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);
    }
    
### 3. Update step
Now that we have incorporated velocity and yaw rate measurement inputs into our filter, we must update particle weights based on LIDAR and RADAR readings of landmarks.

Update step has got three major steps:

- Transformation
- Association
- Update Weights

#### Transformation

We will first need to transform the car’s measurements from its local car coordinate system to the map’s coordinate system.

<img src="https://user-images.githubusercontent.com/677133/232331766-cc75516e-5e72-4425-aead-4c26a5ec6f08.png" width="350">


Map with car observations and particle
Observations in the car coordinate system can be transformed into map coordinates (xm​ and ym​) by passing car observation coordinates (xc​ and yc​), map particle coordinates (xp​ and yp​), and our rotation angle (-90 degrees) through a homogenous transformation matrix. This homogenous transformation matrix, shown below, performs rotation and translation.

<img src="https://user-images.githubusercontent.com/677133/232331788-58cd35b6-3ec6-4c46-9556-7404d6f365df.png" width="350">


Matrix multiplication results in:

<img src="https://user-images.githubusercontent.com/677133/232331795-819e585d-0c73-40a9-8480-387bd4ea567d.png" width="350">



#### Associations

Association is the problem of matching landmark measurement to object in the real world such as map landmarks
<img src="https://user-images.githubusercontent.com/677133/232332205-95757150-da00-4af6-8ab9-7817dc64374c.png" width="350">


Now that observations have been transformed into the map’s coordinate space, the next step is to associate each transformed observation with a land mark identifier. In the map exercise above we have 5 total landmarks each identified as L1, L2, L3, L4, L5, and each with a known map location. We need to associate each transformed observation TOBS1, TOBS2, TOBS3 with one of these 5 identifiers. To do this we must associate the closest landmark to each transformed observation. Consider below example to explain about the data association problem.

<img src="https://user-images.githubusercontent.com/677133/232335935-857b7472-a90d-418c-aed3-035ca8fe1b40.png" width="350">

In this case, we have two Lidar measurements to the rock. We need to find which of these two measurements are corresponding to the rock. If we estimate that, any of the measurement is true, position of the car will be different based on the which measurement we picked.

<img src="https://user-images.githubusercontent.com/677133/232335967-37da55f5-d721-4300-92cb-340a3fea13e1.png" width="350"


As we have multiple measurements to the landmark, we can use Nearest Neighbour technique to find the right one.
<img src="https://user-images.githubusercontent.com/677133/232335990-333b8d4d-520f-490e-887d-c903440f1fed.png" width="350"

In this method, we take the closest measurement as the right measurement.

<img src="https://user-images.githubusercontent.com/677133/232336131-f2ea8e34-74dc-4480-8422-f08c54e9199b.png" width="350"

There are few pros and cons for this approach. Easy to understand and implement is the major pros of this algorithm. In terms cons, not robust to sensor noise, not robust to errors in position estimates etc.

#### Update Weights

Now we that we have done the measurement transformations and associations, we have all the pieces we need to calculate the particle’s final weight. The particles final weight will be calculated as the product of each measurement’s Multivariate-Gaussian probability density.

The Multivariate-Gaussian probability density has two dimensions, x and y. The mean of the Multivariate-Gaussian is the measurement’s associated landmark position and the Multivariate-Gaussian’s standard deviation is described by our initial uncertainty in the x and y ranges. The Multivariate-Gaussian is evaluated at the point of the transformed measurement’s position. The formula for the Multivariate-Gaussian can be seen below.

<img src="https://user-images.githubusercontent.com/677133/232336372-0cc2c399-798d-43ea-9986-18492f0020fa.png" width="350"



And final code for the update step:

    //Each particle for loop
    for(int i = 0; i < num_particles; i++) {
        double paricle_x = particles[i].x;
        double paricle_y = particles[i].y;
        double paricle_theta = particles[i].theta;

        //Create a vector to hold the map landmark locations predicted to be within sensor range of the particle
        vector<LandmarkObs> predictions;

        //Each map landmark for loop
        for(unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {

            //Get id and x,y coordinates
            float lm_x = map_landmarks.landmark_list[j].x_f;
            float lm_y = map_landmarks.landmark_list[j].y_f;
            int lm_id = map_landmarks.landmark_list[j].id_i;

            //Only consider landmarks within sensor range of the particle (rather than using the "dist" method considering a circular region around the                 particle, this considers a rectangular region but is computationally faster)
            if(fabs(lm_x - particle_x) <= sensor_range && fabs(lm_y - particle_y) <= sensor_range) {
                predictions.push_back(LandmarkObs{ lm_id, lm_x, lm_y });
            }
        }

        //Create and populate a copy of the list of observations transformed from vehicle coordinates to map coordinates
        vector<LandmarkObs> trans_os;
        for(unsigned int j = 0; j < observations.size(); j++) {
            double t_x = cos(paricle_theta)*observations[j].x - sin(paricle_theta)*observations[j].y + paricle_x;
            double t_y = sin(paricle_theta)*observations[j].x + cos(paricle_theta)*observations[j].y + paricle_y;
            trans_os.push_back(LandmarkObs{ observations[j].id, t_x, t_y });
        }

        //Data association for the predictions and transformed observations on current particle
        dataAssociation(predictions, trans_os);
        particles[i].weight = 1.0;
        for(unsigned int j = 0; j < trans_os.size(); j++) {
            double o_x, o_y, pr_x, pr_y;
            o_x = trans_os[j].x;
            o_y = trans_os[j].y;
            int asso_prediction = trans_os[j].id;

            //x,y coordinates of the prediction associated with the current observation
            for(unsigned int k = 0; k < predictions.size(); k++) {
                if(predictions[k].id == asso_prediction) {
                    pr_x = predictions[k].x;
                    pr_y = predictions[k].y;
                }
            }

            //Weight for this observation with multivariate Gaussian
            double s_x = std_landmark[0];
            double s_y = std_landmark[1];
            double obs_w = ( 1/(2*M_PI*s_x*s_y)) * exp( -( pow(pr_x-o_x,2)/(2*pow(s_x, 2)) + (pow(pr_y-o_y,2)/(2*pow(s_y, 2))) ) );

            //Product of this obersvation weight with total observations weight
            particles[i].weight *= obs_w;
        }
    }
    
    
### 4. Resample step
Resampling is the technique used to randomly drawing new particles from old ones with replacement in proportion to their importance weights. After resampling, particles with higher weights likely to stay and all others may die out. This is the final step of the Particle filter.

Here is the code for Resample step:

    //Get weights and max weight.
    vector<double> weights;
    double maxWeight = numeric_limits<double>::min();
    for(int i = 0; i < num_particles; i++) {
        weights.push_back(particles[i].weight);
        if(particles[i].weight > maxWeight) {
            maxWeight = particles[i].weight;
        }
    }

    uniform_real_distribution<double> distDouble(0.0, maxWeight);
    uniform_int_distribution<int> distInt(0, num_particles - 1);
    int index = distInt(gen);
    double beta = 0.0;
    vector<Particle> resampledParticles;
    for(int i = 0; i < num_particles; i++) {
        beta += distDouble(gen) * 2.0;
        while(beta > weights[index]) {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        resampledParticles.push_back(particles[index]);
    }

    particles = resampledParticles;






