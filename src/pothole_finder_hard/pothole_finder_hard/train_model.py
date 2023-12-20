from ultralytics import YOLO
import os

wd = os.path.dirname(__file__)

class ModelTrainer:
    def __init__(self):
        model_name = 'yolov8s.pt'
        model = self.load_model(wd + '/' + '../yolo/weights/' + model_name)
        print('Model loaded')
        results = self.train_model(model, model_name.split('.')[0] + '_trained')
        print('Model trained')
        model.export()

    def load_model(self, yolo_model = wd + '/' + '../yolo/weights/yolov8s.pt'):
        print('Loading model...')
        return YOLO(yolo_model)

    def train_model(self, model, name):
        print('Training model...')
        return model.train(data = wd + '/' + '../yolo/data.yaml', epochs = 10, imgsz = 640, batch = 16, workers = 6, val=False, name=name)


def main():
    print('main')
    model_trainer = ModelTrainer()

if __name__ == '__main__':
    main()