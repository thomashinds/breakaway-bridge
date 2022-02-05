/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include "ble-trainer.h"
#include "event-handler.h"
#include "peloton-serial.h"

void ble_task(void *) {}

static BLETrainer ble_trainer;
static EventHandler event_handler(ble_trainer);
static PelotonSerial peloton_serial(event_handler);

extern "C" void app_main(void) {
    // TaskHandle_t ble_task_handle;


    vTaskDelay(pdMS_TO_TICKS(1000));


    // uart_stuff();

    // while (true) {
    //     event_handler.SubmitEvent(PowerReading{500});
    //     vTaskDelay(pdMS_TO_TICKS(300));
    // }
}
