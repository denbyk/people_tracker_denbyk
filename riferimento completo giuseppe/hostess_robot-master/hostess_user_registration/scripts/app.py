#!/usr/bin/python
import eventlet
eventlet.monkey_patch()
from flask import Flask, render_template, request, redirect, url_for
from flask.ext.wtf import Form
from wtforms import StringField, SelectField, FloatField
from flask_bootstrap import Bootstrap
from flask.ext.sqlalchemy import SQLAlchemy
from sqlalchemy import func
from sqlalchemy.sql.schema import UniqueConstraint
from flask_socketio import emit, disconnect, SocketIO
import json, roslib, rospy, actionlib, yaml, os
from cob_people_detection.msg import addDataAction, addDataGoal, deleteDataAction, deleteDataGoal, loadModelAction, loadModelGoal
from collections import namedtuple

f = open(os.path.join(os.path.abspath(os.path.dirname(__file__)), 'config/map.yaml'))
map = yaml.safe_load(f)
f.close()

app = Flask(__name__)

app.config.from_pyfile('config/config.py', silent=False)

app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///' + os.environ['HOME'] + '/.ros/hostess_user_registration/database.sqlite'

db = SQLAlchemy(app)
Bootstrap(app)
socketio = SocketIO(app, async_mode='eventlet')

class User(db.Model):
    __tablename__ = 'users'
    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(64))
    surname = db.Column(db.String(64))
    goal_id = db.Column(db.Integer, db.ForeignKey('goals.id'))
    email = db.Column(db.String(64), unique=True, index=True)

    def __repr__(self):
        return '<User %r>' % self.name

class Goal(db.Model):
    __tablename__ = 'goals'
    id = db.Column(db.Integer, primary_key=True)
    label = db.Column(db.String(64))
    x = db.Column(db.Float)
    y = db.Column(db.Float)
    users = db.relationship('User', backref='goal')

    def __repr__(self):
        return '<User %r>' % self.label

class RegistrationForm(Form):
    name = StringField('Nome')
    surname = StringField('Cognome')
    mail = StringField('E-Mail')
    goal = SelectField('Dove vuoi andare?', choices=[], coerce=int)
    
    def __init__(self, *args, **kwargs):
        Form.__init__(self, *args, **kwargs)
    
    def validate(self):
        outcome = True
        
        rv = Form.validate(self)
        if not rv:
            outcome = False
        
        if self.name.data == '':
            self.name.errors.append('Campo obbligatorio')
            outcome = False
            
        if self.surname.data == '':
            self.surname.errors.append('Campo obbligatorio')
            outcome = False
            
        if self.mail.data == '':
            self.mail.errors.append('Campo obbligatorio')
            outcome = False
        else:
            mail = User.query.filter_by(email=self.mail.data).first()
            if mail is not None:
                self.mail.errors.append('E-Mail presente in database, inserirne una differente')
                outcome = False
        
        found = False
        for v, _ in self.goal.choices:
            if self.goal.data == v:
                found = True
                break
        
        if found == False:
            del self.goal.errors[:]
            self.goal.errors.append('Seleziona una destinazione')

        return outcome


class AddGoalForm(Form):
    label = StringField('Destinazione')
    x = StringField('X')
    y = StringField('Y')
    
    def __init__(self, *args, **kwargs):
        Form.__init__(self, *args, **kwargs)
    
    def validate(self):
        outcome = True
        
        rv = Form.validate(self)
        if not rv:
            outcome = False
        
        if self.label.data == '':
            self.label.errors.append('Campo obbligatorio')
            outcome = False
        
        if self.x.data == '':
            self.x.errors.append('Campo obbligatorio')
            outcome = False
        else:
            try:
                float(self.x.data) if '.' in self.x.data else int(self.x.data)
            except ValueError:
                self.x.errors.append('Valore immesso non valido')
                outcome = False
            
        if self.y.data == '':
            self.y.errors.append('Campo obbligatorio')
            outcome = False
        else:
            try:
                float(self.y.data) if '.' in self.y.data else int(self.y.data)
            except ValueError:
                self.y.errors.append('Valore immesso non valido')
                outcome = False

        return outcome

@app.route('/goals')
def goals():
    return render_template('goals.html', goals=Goal.query.all())

@app.route('/new_goal', methods=['GET', 'POST'])
def new_goal():
    form = AddGoalForm(request.form)
    if request.method == 'POST' and form.validate():
        goal = Goal(label=form.label.data, x=float(form.x.data), y=float(form.y.data))
        with app.app_context():
            db.session.add(goal)
            
        return redirect(url_for('.goals'))
    return render_template('new_goal.html', form=form, map=map)

@app.route('/delete_goals', methods=['GET', 'POST'])
def delete_goal():
    return render_template('delete_goals.html', goals=Goal.query.all())

@app.route('/users')
def users():
    return render_template('users.html', users=User.query.all(), goals=len(Goal.query.all()))

@app.route('/new_user', methods=['GET', 'POST'])
def new_user():
    form = RegistrationForm()
    form.goal.choices = [(g.id, g.label) for g in Goal.query.all()]
    select = 0
    if request.method == 'POST':
        if form.validate():
            return redirect(url_for('.user_calibration'), 307)
        else:
            if form.goal.data is not None:
                select = form.goal.data
    return render_template('new_user.html', form=form, select=select)

@app.route('/new_user/calibration', methods=['POST'])
def user_calibration():
    if request.method == 'POST':
        form = RegistrationForm(request.form)
        goal = Goal.query.filter_by(id=form.goal.data).first_or_404()
        return render_template('user_calibration.html', form=form, goal=goal)
    
@socketio.on('start', namespace='/new_user/calibration')
def start_calibration():
    id = db.session.query(db.func.max(User.id)).scalar()
    if id is None:
        id = 1
    else:
        id = id + 1
        
    id_string = '00000000' + str(id)
    id_string = id_string[-8:]
        
    client = actionlib.SimpleActionClient('/cob_people_detection/face_capture/add_data_server', addDataAction)
    
    if client.wait_for_server(timeout=rospy.Duration(10)):
        goal = addDataGoal(label=id_string, capture_mode=1, continuous_mode_images_to_capture=200, continuous_mode_delay=0.03)
        client.send_goal(goal, done_cb, active_cb, feedback_cb)
    else:
        emit('failed')
    
def done_cb(state, result):
    socketio.emit('calibrated', namespace='/new_user/calibration')
    
def active_cb():
    socketio.emit('started', namespace='/new_user/calibration')
    
def feedback_cb(feedback):
    socketio.emit('progress', {'images_captured': feedback.images_captured}, namespace='/new_user/calibration')

@socketio.on('credentials', namespace='/new_user/calibration')
def save_user(message):
    name=message['nome']
    surname=message['cognome']
    goal_id=message['destinazione']
    email=message['email']
    
    user = User(name=name, surname=surname, goal_id=goal_id, email=email)
    
    with app.app_context():
        db.session.add(user)
    
    emit('saved')
    
@socketio.on('update', namespace='/new_user/calibration')
def update():
    count = User.query.count()
    
    if count > 1:
        client = actionlib.SimpleActionClient('/cob_people_detection/face_recognizer/load_model_server', loadModelAction)
        
        if client.wait_for_server(timeout=rospy.Duration(10)):
            goal = loadModelGoal()
            client.send_goal(goal)
            
            if client.wait_for_result(rospy.Duration(10)):
                emit('updated', {'status': 0})
            else:
                emit('updated', {'status': 2})
    else:
        emit('updated', {'status': 1})

@app.route('/delete_users')
def delete_user():
    return render_template('delete_users.html', users=User.query.all())

@app.route('/delete_user_entries', methods=['POST'])
def delete_user_entries():
    if request.method == 'POST' and request.headers['Content-Type'] == 'application/json; charset=UTF-8':
        client = actionlib.SimpleActionClient('/cob_people_detection/face_capture/delete_data_server', deleteDataAction)
        
        if client.wait_for_server(timeout=rospy.Duration(10)):
            message = json.loads(request.data)
            
            for i in message['utenti']:
                id_string = '00000000' + str(i)
                id_string = id_string[-8:]
                
                goal=deleteDataGoal(delete_mode=2, label=id_string)
                
                client.send_goal(goal)
                client.wait_for_result()
                
                user = User.query.filter_by(id=i).first_or_404()
                
                with app.app_context():
                    db.session.delete(user)
        
            return 'ok'
        return 'failed'
    else:
        return 'bad'
    
@app.route('/delete_goal_entries', methods=['POST'])
def delete_goal_entries():
    if request.method == 'POST' and request.headers['Content-Type'] == 'application/json; charset=UTF-8':
        message = json.loads(request.data)
        for i in message['destinazioni']:
            goal = Goal.query.filter_by(id=i).first_or_404()
            
            with app.app_context():
                db.session.delete(goal)
                
        return 'ok'
    else:
        return 'bad'

@app.route('/')
def root():
    return redirect(url_for('.index'))

@app.route('/index')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    rospy.init_node('hostess_management', disable_signals=True)
    socketio.run(app=app, host='0.0.0.0', port=5000)
