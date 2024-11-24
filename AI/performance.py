from ultralytics import YOLO

model = YOLO('best4.pt')

metrics = model.val(data='/Users/bagseonghyeon/Desktop/haring/AI/dataset/data.yaml')  

# print(metrics)

print(f"Precision: {metrics.box.map50:.4f}")  
print(f"Recall: {metrics.box.map50:.4f}")     
print(f"F1-score: {metrics.box.map:.4f}") 
print(f"Accuracy: {metrics.box.map50:.4f}")     