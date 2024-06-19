/* USING THIS ON DB WILL DELETE PREVIOUS VALUES */

DROP TABLE IF EXISTS MotorData;
DROP TABLE IF EXISTS GPSData;
DROP TABLE IF EXISTS WindData;
DROP TABLE IF EXISTS IMUData;
DROP TABLE IF EXISTS LPPData;


/* Motor Schema */

/* SQL Command to insert data: */
/* INSERT INTO MotorData (motor_id, curr_pos, set_pos) VALUES (1, 10, 10); */

CREATE TABLE IF NOT EXISTS MotorData (
	timestamp DATETIME DEFAULT(STRFTIME('%Y-%m-%d %H:%M:%f', 'now')),
	session_id VARCHAR(50),

	motor_name  VARCHAR(50),
	angle  NUMERIC,
	raw_angle	  NUMERIC, /* raw values */
	PRIMARY KEY (timestamp, motor_name)
);

/* GPS Schema */

/* SQL Command to insert data: */
/* INSERT INTO GPSData (satellites, fix, quality, longitude, latitude, angle, speed, alt_cm, rec_time, rec_date) VALUES (3, 1, 1, 0.0, 0.0, 0.14, 161.23, 275.8, '20:24:11.0','16/4/2012'); */

CREATE TABLE IF NOT EXISTS GPSData (
	timestamp DATETIME DEFAULT(STRFTIME('%Y-%m-%d %H:%M:%f', 'now')),
	session_id VARCHAR(50),

	satellites INTEGER,
	fix INTEGER,
	quality INTEGER,
	longitude NUMERIC,
	latitude NUMERIC,
	angle NUMERIC,
	speed NUMERIC,
	alt_cm NUMERIC,
	rec_time TIMESTAMP,
	rec_date DATE,
	PRIMARY KEY (timestamp)
);

/* Wind Vane and Anemometer Schema */

/* SQL Command to insert data: */
/* INSERT INTO WindData (wind_speed, wind_direction, vane_raw, anem_raw) VALUES (1.2, 234.5, 5.6, 7.8); */

CREATE TABLE IF NOT EXISTS WindData (
	timestamp DATETIME DEFAULT(STRFTIME('%Y-%m-%d %H:%M:%f', 'now')),
	session_id VARCHAR(50),
	
	wind_speed  	NUMERIC,
	wind_direction  NUMERIC,
	vane_raw NUMERIC,
	anem_raw NUMERIC,
	PRIMARY KEY (timestamp)
);

/* IMU Schema */

/* SQL Command to insert data: */
/* INSERT INTO IMUData(euler_x, euler_y, euler_z, angvel_x, angvel_y, angvel_z, acelvel_x, acelvel_y, acelvel_z, magn_x, magn_y, magn_z,lina_x,lina_y,lina_z, grav_x,grav_y,grav_z,temperature) VALUES (1, 2, 3, 4, 5 ,6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19)*/

CREATE TABLE IF NOT EXISTS IMUData (
	timestamp DATETIME DEFAULT(STRFTIME('%Y-%m-%d %H:%M:%f', 'now')),
	session_id VARCHAR(50),
	
	euler_x NUMERIC,
	euler_y NUMERIC,
	euler_z NUMERIC,

	angvel_x NUMERIC,
	angvel_y NUMERIC,
	angvel_z NUMERIC,

	acelvel_x NUMERIC,
	acelvel_y NUMERIC,
	acelvel_z NUMERIC,

	magn_x NUMERIC,
	magn_y NUMERIC,
	magn_z NUMERIC,

	lina_x NUMERIC,
	lina_y NUMERIC,
	lina_z NUMERIC,

	grav_x NUMERIC,
	grav_y NUMERIC,
	grav_z NUMERIC,

	temperature NUMERIC,
	PRIMARY KEY (timestamp)
);

/* LPP Schema */

/* SQL Command to insert data: */
/* INSERT INTO LPPData (current_waypoint_latitude, current_waypoint_longitude, distance_to_waypoint, latitude, longitude, desired_heading) VALUES (1, 10, 10); */

CREATE TABLE IF NOT EXISTS LPPData (
	timestamp DATETIME DEFAULT(STRFTIME('%Y-%m-%d %H:%M:%f', 'now')),
	session_id VARCHAR(50),

	current_waypoint_latitude NUMERIC,
	current_waypoint_longitude NUMERIC,
	distance_to_waypoint NUMERIC,
	latitude NUMERIC,
	longitude NUMERIC,
	desired_heading NUMERIC,

	PRIMARY KEY (timestamp)
);
