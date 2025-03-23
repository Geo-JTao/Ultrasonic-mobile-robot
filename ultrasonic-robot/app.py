from flask import Flask;

app = Flask(__name__)

@app.route("/diagnose")
def diagnose(pos: str):
    start_diagnose();
    
app.run()