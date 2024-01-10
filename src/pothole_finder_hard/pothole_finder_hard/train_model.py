'''
Title: train_model.py
Author: George Davies
email: 27421138@students.lincoln.ac.uk

This file is used to train the YOLOv8 model. 
'''
from ultralytics import YOLO
import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--evolve', type=bool, default=False, help='Evolve model')

args = parser.parse_args()

wd = os.path.dirname(__file__)

class ModelTrainer:
    def __init__(self):
        model_name = 'yolov8l.pt'
        model = self.load_model(wd + '/../yolo/weights/' + model_name)
        print('Model loaded')
        results = None
        if args.evolve:
            results = self.evolve_model(model, model_name.split('.')[0] + '_evolved')
        else:
            results = self.train_model(model, model_name.split('.')[0] + '_trained')
        print('Model trained')
        model.export()

    def load_model(self, yolo_model = wd + '/../yolo/weights/yolov8s.pt'):
        print('Loading model...')
        return YOLO(yolo_model)

    def train_model(self, model, name):
        print('Training model...')
        return model.train(data = wd + '/../yolo/data.yaml', epochs = 300, imgsz = 640, batch = 10, workers = 6, name=name)

    def evolve_model(self, model, name):
        print('Evolving model...')
        res = model.train(data = wd + '/../yolo/data.yaml', epochs = 100, batch = 10, imgsz = 640, workers = 8, name = name)
        new_model = self.load_model(wd + '/../yolo/runs/detect/' + name + '/weights/best.pt')
        return new_model.tune(data = wd + '/../yolo/data.yaml', epochs = 10, iterations = 300, val = False, name = name, batch = 8, workers = 8)



def main():
    print('main')
    model_trainer = ModelTrainer()

if __name__ == '__main__':
    main()