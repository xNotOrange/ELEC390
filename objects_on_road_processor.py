import cv2
import logging
import time
import edgetpu.detection.engine
from PIL import Image
from traffic_objects import Person, OneWayLeft, OneWayRight, StopSign, DoNotEnter

_SHOW_IMAGE = False

class ObjectsOnRoadProcessor(object):
    """
    This class detects objects on the road (e.g. pedestrians, one-way signs, stop signs, do not enter)
    using a TFLite model compiled for the Coral Edge TPU, and then adjusts the car's state accordingly.
    """

    def __init__(self,
                 car=None,
                 speed_limit=40,
                 model='efficientdet-lite-road-objects_edgetpu.tflite',  # Use the Edge TPU-compiled model
                 label='road-objects-labels.txt',
                 width=640,
                 height=480):
        logging.info('Creating an ObjectsOnRoadProcessor...')
        self.width = width
        self.height = height

        # Initialize car state variables
        self.car = car
        self.speed_limit = speed_limit
        self.speed = speed_limit

        # Load labels from file. Expect one label per line.
        with open(label, 'r') as f:
            self.labels = {i: line.strip() for i, line in enumerate(f.readlines())}
        logging.info('Labels loaded: %s', self.labels)

        # Initialize the Edge TPU detection engine.
        logging.info('Initializing Edge TPU with model %s...', model)
        self.engine = edgetpu.detection.engine.DetectionEngine(model)
        self.min_confidence = 0.30
        self.num_of_objects = 3
        logging.info('Edge TPU initialization complete.')

        # OpenCV drawing settings.
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.bottomLeftCornerOfText = (10, height - 10)
        self.fontScale = 1
        self.fontColor = (255, 255, 255)  # white
        self.boxColor = (0, 0, 255)         # red
        self.boxLineWidth = 1
        self.lineType = 2
        self.time_to_show_prediction = 1.0

        # Map label indices to traffic object processors.
        # Label index mapping (order in road-objects-labels.txt):
        # 0: "Pedestrian"  -> Person()
        # 1: "OneWayLeft"  -> OneWayLeft()
        # 2: "OneWayRight" -> OneWayRight()
        # 3: "StopSign"    -> StopSign()
        # 4: "DoNotEnter"  -> DoNotEnter()
        self.traffic_objects = {
            0: Person(),
            1: OneWayLeft(),
            2: OneWayRight(),
            3: StopSign(),
            4: DoNotEnter()
        }

    def process_objects_on_road(self, frame):
        # detect objects with the edge tpr model, draw bouding boxes, update car_state dictionary, and return annotated frame
    
        logging.debug('Processing objects on road...')
        objects, annotated_frame = self.detect_objects(frame)
        self.control_car(objects)
        logging.debug('Finished processing objects.')
        return annotated_frame

    def control_car(self, objects):
        # updates the car_state based on detected objects
        logging.debug('Controlling car based on detections...')
        car_state = {"speed": self.speed_limit, "speed_limit": self.speed_limit}

        if not objects:
            logging.debug('No objects detected; driving at speed limit %s.', self.speed_limit)

        contain_stop_sign = False
        for obj in objects:
            label = self.labels.get(obj.label_id, "Unknown")
            processor = self.traffic_objects.get(obj.label_id)
            if processor is None:
                logging.debug("No processor for label id %s.", obj.label_id)
                continue

            if processor.is_close_by(obj, self.height):
                processor.set_car_state(car_state)
            else:
                logging.debug("[%s] detected but too far; ignoring.", label)

            if label == 'StopSign':
                contain_stop_sign = True

        # If no stop sign is detected, clear the stop sign state.
        if not contain_stop_sign:
            stop_processor = self.traffic_objects.get(3)
            if stop_processor:
                stop_processor.clear()

        self.resume_driving(car_state)

    def resume_driving(self, car_state):
        # update internal speed variables based on car_state and adjust the vehicle's speed
  
        old_speed = self.speed
        self.speed_limit = car_state['speed_limit']
        self.speed = car_state['speed']

        if self.speed == 0:
            self.set_speed(0)
        else:
            self.set_speed(self.speed_limit)
        logging.debug('Speed updated from %d to %d.', old_speed, self.speed)

        if self.speed == 0:
            logging.debug('Vehicle stopped for 1 second.')
            time.sleep(1)

    def set_speed(self, speed):
        # sets the car speed
        self.speed = speed
        if self.car is not None:
            logging.debug("Setting car speed to %d.", speed)
            self.car.back_wheels.speed = speed

    def detect_objects(self, frame):
        # uses the tpu to detect objects
        logging.debug('Detecting objects using Edge TPU...')
        start_time = time.time()
        frame_RGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_pil = Image.fromarray(frame_RGB)
        objects = self.engine.DetectWithImage(img_pil,
                                              threshold=self.min_confidence,
                                              keep_aspect_ratio=True,
                                              relative_coord=False,
                                              top_k=self.num_of_objects)
        if objects:
            for obj in objects:
                box = obj.bounding_box
                width = box[1][0] - box[0][0]
                height_box = box[1][1] - box[0][1]
                logging.debug("%s: %.0f%%, width=%.0f, height=%.0f" %
                              (self.labels[obj.label_id], obj.score * 100, width, height_box))
                top_left = (int(box[0][0]), int(box[0][1]))
                bottom_right = (int(box[1][0]), int(box[1][1]))
                cv2.rectangle(frame, top_left, bottom_right, self.boxColor, self.boxLineWidth)
                annotation = "%s %.0f%%" % (self.labels[obj.label_id], obj.score * 100)
                text_position = (top_left[0], top_left[1] + 15)
                cv2.putText(frame, annotation, text_position, self.font, self.fontScale, self.boxColor, self.lineType)
        else:
            logging.debug('No objects detected.')

        elapsed_time = time.time() - start_time
        fps_text = "%.1f FPS" % (1.0 / elapsed_time) if elapsed_time > 0 else "FPS: N/A"
        cv2.putText(frame, fps_text, self.bottomLeftCornerOfText, self.font, self.fontScale, self.fontColor, self.lineType)

        return objects, frame
