import numpy as np
from sklearn.svm import SVC
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import make_pipeline
import joblib

# =========================
# TRAINING DATA SIMULATION
# Features: [x, z, angle]
# x     = horizontal offset
# z     = distance to AprilTag
# angle = orientation error
# =========================

X = []
y = []

for _ in range(300):
    x = np.random.uniform(-0.03, 0.03)
    z = np.random.uniform(0.20, 0.35)
    angle = np.random.uniform(-5, 5)
    X.append([x, z, angle])
    y.append("DOCK")

for _ in range(300):
    x = np.random.uniform(-0.25, 0.25)
    z = np.random.uniform(0.35, 1.20)
    angle = np.random.uniform(-30, 30)
    X.append([x, z, angle])
    y.append("ADJUST")

for _ in range(300):
    x = np.random.uniform(-0.40, 0.40)
    z = np.random.uniform(1.20, 2.50)
    angle = np.random.uniform(-45, 45)
    X.append([x, z, angle])
    y.append("APPROACH")

X = np.array(X)
y = np.array(y)

# SVM model
model = make_pipeline(
    StandardScaler(),
    SVC(kernel="rbf", C=10, gamma="scale")
)

model.fit(X, y)

joblib.dump(model, "svm_docking_model.pkl")

print("SVM model saved as svm_docking_model.pkl")