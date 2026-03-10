/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

/*
 * Description:   Example of a traffic light system in crossroads.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include <webots/led.h>
#include <webots/robot.h>
#include <webots/emitter.h>

#define TIME_STEP 64
#define N_LIGHT 12


typedef struct {
  char id[32];
  char states[N_LIGHT];
} TrafficLightMessage;

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag red_light[N_LIGHT], orange_light[N_LIGHT], green_light[N_LIGHT];
  char red_light_string[32];
  char orange_light_string[32];
  char green_light_string[32];
  int i;
  for (i = 0; i < N_LIGHT; i++) {
    snprintf(red_light_string, 32, "red light %d", i);
    snprintf(orange_light_string, 32, "orange light %d", i);
    snprintf(green_light_string, 32, "green light %d", i);
    red_light[i] = wb_robot_get_device(red_light_string);
    orange_light[i] = wb_robot_get_device(orange_light_string);
    green_light[i] = wb_robot_get_device(green_light_string);
  }

  int min = -100;
  int max = 100;
  srand(time(NULL) + atoi(wb_robot_get_name()));
  int offset = atoi(wb_robot_get_name()) % 654;
  int t = offset * TIME_STEP;

  wb_robot_step(TIME_STEP); // only fetch emitter after device tree is ready
  WbDeviceTag emitter = wb_robot_get_device("traffic_emitter");
  if (emitter == 0) {
    printf("Emitter not found!\n");
  } else {
    printf("Found emitter for TL_%s!\n", wb_robot_get_name());
  }

  static char prev[N_LIGHT] = {0};
  while (wb_robot_step(TIME_STEP) != -1) {
    t += TIME_STEP;

    if (t == TIME_STEP) {
      wb_led_set(green_light[0], 1);
      wb_led_set(green_light[1], 1);
      wb_led_set(green_light[6], 1);
      wb_led_set(green_light[7], 1);
      wb_led_set(red_light[3], 1);
      wb_led_set(red_light[4], 1);
      wb_led_set(red_light[9], 1);
      wb_led_set(red_light[10], 1);
      wb_led_set(red_light[2], 1);
      wb_led_set(red_light[8], 1);
      wb_led_set(red_light[5], 1);
      wb_led_set(red_light[11], 1);
    }
    if (t == 125 * TIME_STEP) {
      wb_led_set(green_light[0], 0);
      wb_led_set(green_light[1], 0);
      wb_led_set(green_light[6], 0);
      wb_led_set(green_light[7], 0);
      wb_led_set(orange_light[0], 1);
      wb_led_set(orange_light[1], 1);
      wb_led_set(orange_light[6], 1);
      wb_led_set(orange_light[7], 1);
    }
    if (t == 156 * TIME_STEP) {
      wb_led_set(orange_light[0], 0);
      wb_led_set(orange_light[1], 0);
      wb_led_set(orange_light[6], 0);
      wb_led_set(orange_light[7], 0);
      wb_led_set(red_light[0], 1);
      wb_led_set(red_light[1], 1);
      wb_led_set(red_light[6], 1);
      wb_led_set(red_light[7], 1);
      wb_led_set(red_light[3], 0);
      wb_led_set(red_light[4], 0);
      wb_led_set(red_light[9], 0);
      wb_led_set(red_light[10], 0);
      wb_led_set(green_light[3], 1);
      wb_led_set(green_light[4], 1);
      wb_led_set(green_light[9], 1);
      wb_led_set(green_light[10], 1);
    }
    if (t == 281 * TIME_STEP) {
      wb_led_set(green_light[3], 0);
      wb_led_set(green_light[4], 0);
      wb_led_set(green_light[9], 0);
      wb_led_set(green_light[10], 0);
      wb_led_set(orange_light[3], 1);
      wb_led_set(orange_light[4], 1);
      wb_led_set(orange_light[9], 1);
      wb_led_set(orange_light[10], 1);
    }
    if (t == 312 * TIME_STEP) {
      wb_led_set(orange_light[3], 0);
      wb_led_set(orange_light[4], 0);
      wb_led_set(orange_light[9], 0);
      wb_led_set(orange_light[10], 0);
      wb_led_set(red_light[3], 1);
      wb_led_set(red_light[4], 1);
      wb_led_set(red_light[9], 1);
      wb_led_set(red_light[10], 1);
      wb_led_set(red_light[2], 0);
      wb_led_set(red_light[8], 0);
      wb_led_set(green_light[2], 1);
      wb_led_set(green_light[8], 1);
    }
    if (t == 467 * TIME_STEP) {
      wb_led_set(green_light[2], 0);
      wb_led_set(green_light[8], 0);
      wb_led_set(orange_light[2], 1);
      wb_led_set(orange_light[8], 1);
    }
    if (t == 498 * TIME_STEP) {
      wb_led_set(orange_light[2], 0);
      wb_led_set(orange_light[8], 0);
      wb_led_set(red_light[2], 1);
      wb_led_set(red_light[8], 1);
      wb_led_set(red_light[5], 0);
      wb_led_set(red_light[11], 0);
      wb_led_set(green_light[5], 1);
      wb_led_set(green_light[11], 1);
    }
    if (t == 623 * TIME_STEP) {
      wb_led_set(green_light[5], 0);
      wb_led_set(green_light[11], 0);
      wb_led_set(orange_light[5], 1);
      wb_led_set(orange_light[11], 1);
    }
    if (t == 654 * TIME_STEP) {
      wb_led_set(orange_light[5], 0);
      wb_led_set(orange_light[11], 0);
      wb_led_set(red_light[0], 0);
      wb_led_set(red_light[1], 0);
      wb_led_set(red_light[6], 0);
      wb_led_set(red_light[7], 0);
      t = 0;
    }

    char light_states[N_LIGHT];
    for (i = 0; i < N_LIGHT; i++) {
      if (wb_led_get(red_light[i])) {
        light_states[i] = 'r';
      } else if (wb_led_get(orange_light[i])) {
        light_states[i] = 'o';
      } else if (wb_led_get(green_light[i])) {
        light_states[i] = 'g';
      } else {
        light_states[i] = 'x';
      }
    }
    
    // only send message if emitter exists and traffic data changes
    if (emitter != 0 && memcmp(prev, light_states, N_LIGHT) != 0) {
      TrafficLightMessage msg;
      snprintf(msg.id, 32, "%s", wb_robot_get_name());
      memcpy(msg.states, light_states, N_LIGHT);
      memcpy(prev, light_states, N_LIGHT);
      wb_emitter_send(emitter, &msg, sizeof(TrafficLightMessage));
    }
  };

  wb_robot_cleanup();

  return 0;
}