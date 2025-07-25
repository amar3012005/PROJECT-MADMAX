#!/home/amar/madmax/madmax/bin/python3.9

from __future__ import print_function

'''
log analysis program
Andrew Tridgell December 2014
'''

import copy
import sys
import time
import os
import fnmatch
import threading
import shlex
import traceback
from math import *

os.environ['MAVLINK20'] = '1'

from MAVProxy.modules.lib import multiproc
from MAVProxy.modules.lib import rline
from MAVProxy.modules.lib import wxconsole
from MAVProxy.modules.lib import param_help
from MAVProxy.modules.lib import param_ftp
from MAVProxy.modules.lib.graph_ui import Graph_UI
from pymavlink.mavextra import *
from MAVProxy.modules.lib.mp_menu import *
import MAVProxy.modules.lib.mp_util as mp_util
from pymavlink import mavutil
from pymavlink import mavwp
from pymavlink import DFReader
from MAVProxy.modules.lib.mp_settings import MPSettings, MPSetting
from MAVProxy.modules.lib import wxsettings
from MAVProxy.modules.lib.graphdefinition import GraphDefinition
from lxml import objectify
import pkg_resources
from builtins import input
import datetime
import matplotlib
import struct

grui = []
flightmodes = None

# Global var to hold the GUI menu element
TopMenu = None

# have we decoded MAVFtp params?
done_ftp_decode = False

def xml_unescape(e):
    '''unescape < amd >'''
    e = e.replace('&gt;', '>')
    e = e.replace('&lt;', '<')
    return e

def xml_escape(e):
    '''escape < amd >'''
    e = e.replace('>', '&gt;')
    e = e.replace('<', '&lt;')
    return e

def timestring(msg):
    '''return string for msg timestamp'''
    ts_ms = int(msg._timestamp * 1000.0) % 1000
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(msg._timestamp)) + ".%.03u" % ts_ms

class MEStatus(object):
    '''status object to conform with mavproxy structure for modules'''
    def __init__(self):
        self.msgs = {}

class XLimits(object):
    '''A class capturing timestamp limits'''
    def __init__(self):
        self.last_xlim = None
        self.xlim_low = None
        self.xlim_high = None

    def timestamp_in_range(self, timestamp):
        '''check if a timestamp is in current limits
        return -1 if too low
        return 1 if too high
        return 0 if in range
        '''
        if self.xlim_low is not None and timestamp < self.xlim_low:
            return -1
        if self.xlim_high is not None and timestamp > self.xlim_high:
            return 1
        return 0    

xlimits = XLimits()

class MEState(object):
    '''holds state of MAVExplorer'''
    def __init__(self):
        self.input_queue = multiproc.Queue()
        self.rl = None
        self.console = wxconsole.MessageConsole(title='MAVExplorer')
        self.exit = False
        self.status = MEStatus()
        self.settings = MPSettings(
            [ MPSetting('marker', str, '+', 'data marker', tab='Graph'),
              MPSetting('condition', str, None, 'condition'),
              MPSetting('xaxis', str, None, 'xaxis'),
              MPSetting('linestyle', str, None, 'linestyle'),
              MPSetting('show_flightmode', int, 1, 'show flightmode'),
              MPSetting('sync_xzoom', bool, True, 'sync X-axis zoom'),
              MPSetting('sync_xmap', bool, True, 'sync X-axis zoom for map'),
              MPSetting('legend', str, 'upper left', 'legend position'),
              MPSetting('legend2', str, 'upper right', 'legend2 position'),
              MPSetting('title', str, None, 'Graph title'),
              MPSetting('debug', int, 0, 'debug level'),
              MPSetting('paramdocs', bool, True, 'show param docs'),
              MPSetting('max_rate', float, 0, 'maximum display rate of graphs in Hz'),
              MPSetting('vehicle_type', str, 'Auto', 'force vehicle type for mode handling'),
              ]
            )

        self.mlog = None
        self.mav_param = None
        self.filename = None
        self.command_map = command_map
        self.completions = {
            "set"       : ["(SETTING)"],
            "condition" : ["(VARIABLE)"],
            "graph"     : ['(VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE)'],
            "dump"      : ['(MESSAGETYPE)', '--verbose (MESSAGETYPE)'],
            "map"       : ['(VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE) (VARIABLE)'],
            "param"     : ['download', 'check', 'help (PARAMETER)', 'save', 'savechanged', 'diff', 'show', 'check'],
            "logmessage": ['download', 'help (MESSAGETYPE)'],
            }
        self.aliases = {}
        self.graphs = []
        self.flightmode_selections = []
        self.last_graph = GraphDefinition(self.settings.title, '', '', [], None)
        
        #pipe to the wxconsole for any child threads (such as the save dialog box)
        self.parent_pipe_recv_console,self.child_pipe_send_console = multiproc.Pipe(duplex=False)
        #pipe for creating graphs (such as from the save dialog box)
        self.parent_pipe_recv_graph,self.child_pipe_send_graph = multiproc.Pipe(duplex=False)
        self.param_help = param_help.ParamHelp()
        
        tConsoleWrite = threading.Thread(target=self.pipeRecvConsole)
        tConsoleWrite.daemon = True
        tConsoleWrite.start()
        tGraphWrite = threading.Thread(target=self.pipeRecvGraph)
        tGraphWrite.daemon = True
        tGraphWrite.start()

    def pipeRecvConsole(self):
        '''watch for piped data from save dialog'''
        try:
            while True:
                console_msg = self.parent_pipe_recv_console.recv()
                if console_msg is not None:
                    self.console.writeln(console_msg)
                time.sleep(0.1)
        except EOFError:
            pass

    def pipeRecvGraph(self):
        '''watch for piped data from save dialog'''
        try:
            while True:
                graph_rec = self.parent_pipe_recv_graph.recv()
                if graph_rec is not None:
                    mestate.input_queue.put(graph_rec)
                time.sleep(0.1)
        except EOFError:
            pass
            
def have_graph(name):
    '''return true if we have a graph of the given name'''
    for g in mestate.graphs:
        if g.name == name:
            return True
    return False

def menu_callback(m):
    '''called on menu selection'''
    if m.returnkey.startswith('# '):
        cmd = m.returnkey[2:]
        if m.handler is not None:
            if m.handler_result is None:
                return
            cmd += m.handler_result
        process_stdin(cmd)
    elif m.returnkey == 'menuSettings':
        wxsettings.WXSettings(mestate.settings)
    elif m.returnkey.startswith("mode-"):
        idx = int(m.returnkey[5:])
        mestate.flightmode_selections[idx] = m.IsChecked()
    elif m.returnkey.startswith("loadLog"):
        print("File: " + m.returnkey[8:])
    elif m.returnkey == 'quit':
        mestate.console.close()
        mestate.exit = True
        print("Exited. Press Enter to continue.")
        sys.exit(0)

    else:
        print('Unknown menu selection: %s' % m.returnkey)


def flightmode_menu():
    '''construct flightmode menu'''
    global flightmodes
    ret = []
    idx = 0
    for (mode,t1,t2) in flightmodes:
        modestr = "%s %us" % (mode, (t2-t1))
        ret.append(MPMenuCheckbox(modestr, modestr, 'mode-%u' % idx))
        idx += 1
        mestate.flightmode_selections.append(False)
    return ret


def graph_menus():
    '''return menu tree for graphs (recursive)'''
    ret = MPMenuSubMenu('Graphs', [])
    for i in range(len(mestate.graphs)):
        g = mestate.graphs[i]
        path = g.name.split('/')
        name = path[-1]
        path = path[:-1]
        ret.add_to_submenu(path, MPMenuItem(name, name, '# graph :%u' % i))
    return ret

def setup_file_menu():
    global TopMenu
    TopMenu = MPMenuTop([])
    TopMenu.add(MPMenuSubMenu('MAVExplorer',
                           items=[MPMenuItem('Settings', 'Settings', 'menuSettings'),
                                  MPMenuItem('&Open\tCtrl+O', 'Open Log', '# loadLog ',
                                            handler=MPMenuCallFileDialog(
                                                                        flags=('open',),
                                                                        title='Logfile Load',
                                                                        wildcard='*.tlog;*.log;*.BIN;*.bin')),
                                  MPMenuItem('&Quit\tCtrl+Q', 'Quit', 'quit')]))
    mestate.console.set_menu(TopMenu, menu_callback)

def setup_menus():
    '''setup console menus'''
    global TopMenu
    TopMenu.add(MPMenuSubMenu('Display',
                           items=[MPMenuItem('Map', 'Map', '# map'),
                                  MPMenuItem('Save Graph', 'Save', '# save'),
                                  MPMenuItem('Reload Graphs', 'Reload', '# reload')]))
    TopMenu.add(graph_menus())
    TopMenu.add(MPMenuSubMenu('FlightMode', items=flightmode_menu()))
    TopMenu.add(MPMenuSubMenu('Tools',
                              items=[MPMenuItem('MagFit', 'MagFit', '# magfit'),
                                     MPMenuItem('Stats', 'Stats', '# stats'),
                                     MPMenuItem('FFT', 'FFT', '# fft')]))

    mestate.console.set_menu(TopMenu, menu_callback)

def expression_ok(expression, msgs=None):
    '''return True if an expression is OK with current messages'''
    expression_ok = True
    if expression is None:
        return False
    fields = expression.split()
    if msgs is None:
        msgs = mestate.status.msgs
    for f in fields:
        try:
            if f.endswith(">"):
                a2 = f.rfind("<")
                if a2 != -1:
                    f = f[:a2]
            if f.endswith(':2'):
                f = f[:-2]
            if f[-1] == '}':
                # avoid passing nocondition unless needed to allow us to work witih older
                # pymavlink versions
                res = mavutil.evaluate_expression(f, msgs, nocondition=True)
            else:
                res = mavutil.evaluate_expression(f, msgs)
            if res is None:
                expression_ok = False
        except Exception:
            expression_ok = False
            break
    return expression_ok

def load_graph_xml(xml, filename, load_all=False):
    '''load a graph from one xml string'''
    ret = []
    try:
        root = objectify.fromstring(xml)
    except Exception as ex:
        print(filename, ex)
        return []
    if root.tag != 'graphs':
        return []
    if not hasattr(root, 'graph'):
        return []
    names = set()
    for g in root.graph:
        name = g.attrib['name']
        expressions = []
        for e in g.expression:
            if e.text is not None:
                expressions.append(e.text)
        if load_all:
            if not name in names:
                if hasattr(g,'description'):
                    description = g.description.text
                else:
                    description = ''
                ret.append(GraphDefinition(name, expressions[0], description, expressions, filename))
            names.add(name)
            continue
        if have_graph(name):
            continue
        for e in expressions:
            if e is None:
                continue
            e = xml_unescape(e)
            if expression_ok(e):
                if hasattr(g,'description'):
                    description = g.description.text
                else:
                    description = ''
                ret.append(GraphDefinition(name, e, description, expressions, filename))
                break
    return ret

def load_graphs():
    '''load graphs from mavgraphs.xml'''
    mestate.graphs = []
    gfiles = ['mavgraphs.xml']
    for dirname, dirnames, filenames in os.walk(mp_util.dot_mavproxy()):
        # Skip XML files in the LogMessages subfolder
        if os.path.basename(dirname) == "LogMessages":
            continue
        for filename in filenames:
            if filename.lower().endswith('.xml'):
                gfiles.append(os.path.join(dirname, filename))

    for file in gfiles:
        if not os.path.exists(file):
            continue
        # skip parameter files.  They specify an encoding, and under
        # Python3 this leads to a warning from etree
        if os.path.basename(file) in ["ArduSub.xml", "ArduPlane.xml", "APMrover2.xml", "ArduCopter.xml", "AntennaTracker.xml", "Blimp.xml", "Rover.xml"]:
            continue
        graphs = load_graph_xml(open(file).read(), file)
        if graphs:
            mestate.graphs.extend(graphs)
            mestate.console.writeln("Loaded %s" % file)
    # also load the built in graphs
    try:
        dlist = pkg_resources.resource_listdir("MAVProxy", "tools/graphs")
        for f in dlist:
            raw = pkg_resources.resource_stream("MAVProxy", "tools/graphs/%s" % f).read()
            graphs = load_graph_xml(raw, None)
            if graphs:
                mestate.graphs.extend(graphs)
                mestate.console.writeln("Loaded %s" % f)
    except Exception:
        #we're in a Windows exe, where pkg_resources doesn't work
        import pkgutil
        for f in ["ekf3Graphs.xml", "ekfGraphs.xml", "mavgraphs.xml", "mavgraphs2.xml"]:
            raw = pkgutil.get_data( 'MAVProxy', 'tools//graphs//' + f)
            graphs = load_graph_xml(raw, None)
            if graphs:
                mestate.graphs.extend(graphs)
                mestate.console.writeln("Loaded %s" % f)
    mestate.graphs = sorted(mestate.graphs, key=lambda g: g.name)

def flightmode_colours():
    '''return mapping of flight mode to colours'''
    from MAVProxy.modules.lib.grapher import flightmode_colours
    mapping = {}
    idx = 0
    for (mode,t0,t1) in flightmodes:
        if not mode in mapping:
            mapping[mode] = flightmode_colours[idx]
            idx += 1
            if idx >= len(flightmode_colours):
                idx = 0
    return mapping

def check_vehicle_type():
    '''check vehicle_type option'''
    vtypes = { "Rover" : mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
               "Plane" : mavutil.mavlink.MAV_TYPE_FIXED_WING,
               "Copter" : mavutil.mavlink.MAV_TYPE_QUADROTOR,
               "Tracker" : mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER,
               "Antenna" : mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER,
               "Sub" : mavutil.mavlink.MAV_TYPE_SUBMARINE,
               "Blimp" : mavutil.mavlink.MAV_TYPE_AIRSHIP
    }
    if mestate.settings.vehicle_type == 'Auto':
        # let mavutil handle it
        return
    old_mav_type = mestate.mlog.mav_type
    for k in vtypes.keys():
        if mestate.settings.vehicle_type.lower().find(k.lower()) != -1:
            mestate.mlog.mav_type = vtypes[k]
            if old_mav_type != mestate.mlog.mav_type:
                global flightmodes
                mestate.mlog._flightmodes = None
                flightmodes = mestate.mlog.flightmode_list()
            return
    print("Unknown vehicle type '%s'" % mestate.settings.vehicle_type)


def cmd_graph(args):
    '''graph command'''
    usage = "usage: graph <FIELD...>"
    if len(args) < 1:
        print(usage)
        return
    check_vehicle_type()
    if args[0][0] == ':':
        i = int(args[0][1:])
        g = mestate.graphs[i]
        expression = g.expression
        args = expression.split()
        mestate.console.write("Added graph: %s\n" % g.name)
        if g.description:
            mestate.console.write("%s\n" % g.description, fg='blue')
        mestate.rl.add_history("graph %s" % ' '.join(expression.split()))
        mestate.last_graph = g
    else:
        expression = ' '.join(args)
        mestate.last_graph = GraphDefinition(mestate.settings.title, expression, '', [expression], None)
    if mestate.settings.debug > 0:
        print("Adding graph: %s" % mestate.last_graph.expression)
    grui.append(Graph_UI(mestate))
    grui[-1].display_graph(mestate.last_graph, flightmode_colours())
    global xlimits
    if xlimits.last_xlim is not None and mestate.settings.sync_xzoom:
        #print("initial: ", xlimits.last_xlim)
        grui[-1].set_xlim(xlimits.last_xlim)

map_timelim_pipes = []

def cmd_map(args):
    '''map command'''
    import mavflightview
    #mestate.mlog.reduce_by_flightmodes(mestate.flightmode_selections)
    #setup and process the map
    check_vehicle_type()
    options = mavflightview.mavflightview_options()
    options.condition = mestate.settings.condition
    options._flightmodes = mestate.mlog._flightmodes
    options.show_flightmode_legend = mestate.settings.show_flightmode
    options.colour_source='flightmode'
    options.nkf_sample = 1
    if len(args) > 0:
        options.types = ':'.join(args)
        if len(options.types) > 1:
            options.colour_source='type'
    mfv_mav_ret = mavflightview.mavflightview_mav(mestate.mlog, options, mestate.flightmode_selections)
    if mfv_mav_ret is None:
        return
    [path, wp, fen, used_flightmodes, mav_type, instances] = mfv_mav_ret
    global map_timelim_pipes
    timelim_pipe = multiproc.Pipe()
    child = multiproc.Process(target=mavflightview.mavflightview_show, args=[path, wp, fen, used_flightmodes, mav_type, options, instances, None, timelim_pipe])
    map_timelim_pipes.append(timelim_pipe)
    global xlimits
    if xlimits.last_xlim is not None and mestate.settings.sync_xmap:
        try:
            timelim_pipe[0].send(xlimits.last_xlim)
        except Exception:
            pass
    child.start()
    mestate.mlog.rewind()

def cmd_set(args):
    '''control MAVExporer options'''
    mestate.settings.command(args)

def cmd_condition(args):
    '''control MAVExporer conditions'''
    if len(args) == 0:
        print("condition is: %s" % mestate.settings.condition)
        return
    mestate.settings.condition = ' '.join(args)
    if len(mestate.settings.condition) == 0 or mestate.settings.condition == 'clear':
        mestate.settings.condition = None

def cmd_reload(args):
    '''reload graphs'''
    mestate.console.writeln('Reloading graphs', fg='blue')
    load_graphs()
    setup_menus()
    mestate.console.write("Loaded %u graphs\n" % len(mestate.graphs))

fft_tool = None

def cmd_fft(args):
    '''display fft from log'''

    from MAVProxy.modules.lib import mav_fft
    if len(args) > 0:
        condition = args[0]
    else:
        condition = None
    global fft_tool, xlimits
    fft_tool = mav_fft.MavFFT(mlog=mestate.mlog,
                              xlimits=xlimits)
    fft_tool.start()

msgstats_tool = None

def cmd_stats(args):
    '''show status on log'''

    from MAVProxy.modules.lib import msgstats
    global msgstats_tool
    msgstats_tool = msgstats.MPMsgStats(mlog=mestate.mlog)
    msgstats_tool.start()

def cmd_dump(args):
    '''dump messages from log'''
    global xlimits

    # understand --verbose to give as much information about message as possible
    verbose = False
    if "--verbose" in args:
        verbose = True
        args = list(filter(lambda x : x != "--verbose", args))

    if len(args) > 0:
        wildcard = args[0]
    else:
        print("Usage: dump PATTERN")
        return
    mlog = mestate.mlog
    mlog.rewind()
    types = []
    for p in wildcard.split(','):
        for t in mlog.name_to_id.keys():
            if fnmatch.fnmatch(t, p):
                types.extend([t])
    while True:
        msg = mlog.recv_match(type=types, condition=mestate.settings.condition)
        if msg is None:
            break
        in_range = xlimits.timestamp_in_range(msg._timestamp)
        if in_range < 0:
            continue
        if in_range > 0:
            continue
        if verbose and "pymavlink.dialects" in str(type(msg)):
            mavutil.dump_message_verbose(sys.stdout, msg)
        elif verbose and hasattr(msg,"dump_verbose"):
            msg.dump_verbose(sys.stdout)
        else:
            print("%s %s" % (timestring(msg), msg))
    mlog.rewind()

mfit_tool = None

def cmd_magfit(args):
    '''fit magnetic field'''

    from MAVProxy.modules.lib import magfit    
    global mfit_tool, xlimits
    mfit_tool = magfit.MagFit(title="MagFit",
                              mlog=mestate.mlog,
                              xlimits=xlimits)
    mfit_tool.start()

def save_graph(graphdef):
    '''save a graph as XML'''
    if graphdef.filename is None:
        graphdef.filename = os.path.join(mp_util.dot_mavproxy(), 'mavgraphs.xml')
    contents = None
    try:
        contents = open(graphdef.filename).read()
        graphs = load_graph_xml(contents, graphdef.filename, load_all=True)
    except Exception as ex:
        graphs = []
        print(ex)
    if contents is not None and len(graphs) == 0:
        print("Unable to parse %s" % graphdef.filename)
        return
    if contents is not None:
        try:
            open(graphdef.filename + ".bak",'w').write(contents)
        except Exception:
            pass
    found_name = False
    for i in range(len(graphs)):
        if graphs[i].name == graphdef.name:
            graphs[i] = graphdef
            found_name = True
            break
    if not found_name:
        graphs.append(graphdef)
    pipe_console_input.send("Saving %u graphs to %s" % (len(graphs), graphdef.filename))
    f = open(graphdef.filename, "w")
    f.write("<graphs>\n\n")
    for g in graphs:
        f.write(" <graph name='%s'>\n" % g.name.strip())
        if g.description is None:
            g.description = ''
        f.write("  <description>%s</description>\n" % g.description.strip())
        for e in g.expressions:
            if e is None:
                continue
            e = xml_escape(e)
            f.write("  <expression>%s</expression>\n" % e.strip())
        f.write(" </graph>\n\n")
    f.write("</graphs>\n")
    f.close()

def save_callback(operation, graphdef):
    '''callback from save thread'''
    if operation == 'test':
        for e in graphdef.expressions:
            if e is None:
                continue
            if expression_ok(e, msgs):
                graphdef.expression = e
                pipe_graph_input.send('graph ' + graphdef.expression)
                return
        pipe_console_input.send('Invalid graph expressions')
        return
    if operation == 'save':
        save_graph(graphdef)

def save_process(MAVExpLastGraph, child_pipe_console_input, child_pipe_graph_input, statusMsgs):
    '''process for saving a graph'''
    from MAVProxy.modules.lib import wx_processguard
    from MAVProxy.modules.lib.wx_loader import wx
    from MAVProxy.modules.lib.wxgrapheditor import GraphDialog
    
    #This pipe is used to send text to the console
    global pipe_console_input
    pipe_console_input = child_pipe_console_input

    #This pipe is used to send graph commands
    global pipe_graph_input
    pipe_graph_input = child_pipe_graph_input
    
    #The valid expression messages, required to
    #validate the expression in the dialog box
    global msgs
    msgs = statusMsgs
    
    app = wx.App(False)
    if MAVExpLastGraph.description is None:
        MAVExpLastGraph.description = ''
    frame = GraphDialog('Graph Editor',
                        MAVExpLastGraph,
                        save_callback)
    frame.ShowModal()
    frame.Destroy()

def cmd_save(args):
    '''save a graph'''
    child = multiproc.Process(target=save_process, args=[mestate.last_graph, mestate.child_pipe_send_console, mestate.child_pipe_send_graph, mestate.status.msgs])
    child.start()
    
# events from EV messages, taken from AP_Logger.h
events = {
    7 : "DATA_AP_STATE",
    8 : "DATA_SYSTEM_TIME_SET",
    9 : "DATA_INIT_SIMPLE_BEARING",
    10 : "DATA_ARMED",
    11 : "DATA_DISARMED",
    15 : "DATA_AUTO_ARMED",
    17 : "DATA_LAND_COMPLETE_MAYBE",
    18 : "DATA_LAND_COMPLETE",
    28 : "DATA_NOT_LANDED",
    19 : "DATA_LOST_GPS",
    21 : "DATA_FLIP_START",
    22 : "DATA_FLIP_END",
    25 : "DATA_SET_HOME",
    26 : "DATA_SET_SIMPLE_ON",
    27 : "DATA_SET_SIMPLE_OFF",
    29 : "DATA_SET_SUPERSIMPLE_ON",
    30 : "DATA_AUTOTUNE_INITIALISED",
    31 : "DATA_AUTOTUNE_OFF",
    32 : "DATA_AUTOTUNE_RESTART",
    33 : "DATA_AUTOTUNE_SUCCESS",
    34 : "DATA_AUTOTUNE_FAILED",
    35 : "DATA_AUTOTUNE_REACHED_LIMIT",
    36 : "DATA_AUTOTUNE_PILOT_TESTING",
    37 : "DATA_AUTOTUNE_SAVEDGAINS",
    38 : "DATA_SAVE_TRIM",
    39 : "DATA_SAVEWP_ADD_WP",
    41 : "DATA_FENCE_ENABLE",
    42 : "DATA_FENCE_DISABLE",
    43 : "DATA_ACRO_TRAINER_DISABLED",
    44 : "DATA_ACRO_TRAINER_LEVELING",
    45 : "DATA_ACRO_TRAINER_LIMITED",
    46 : "DATA_GRIPPER_GRAB",
    47 : "DATA_GRIPPER_RELEASE",
    49 : "DATA_PARACHUTE_DISABLED",
    50 : "DATA_PARACHUTE_ENABLED",
    51 : "DATA_PARACHUTE_RELEASED",
    52 : "DATA_LANDING_GEAR_DEPLOYED",
    53 : "DATA_LANDING_GEAR_RETRACTED",
    54 : "DATA_MOTORS_EMERGENCY_STOPPED",
    55 : "DATA_MOTORS_EMERGENCY_STOP_CLEARED",
    56 : "DATA_MOTORS_INTERLOCK_DISABLED",
    57 : "DATA_MOTORS_INTERLOCK_ENABLED",
    58 : "DATA_ROTOR_RUNUP_COMPLETE",
    59 : "DATA_ROTOR_SPEED_BELOW_CRITICAL",
    60 : "DATA_EKF_ALT_RESET",
    61 : "DATA_LAND_CANCELLED_BY_PILOT",
    62 : "DATA_EKF_YAW_RESET",
    63 : "DATA_AVOIDANCE_ADSB_ENABLE",
    64 : "DATA_AVOIDANCE_ADSB_DISABLE",
    65 : "DATA_AVOIDANCE_PROXIMITY_ENABLE",
    66 : "DATA_AVOIDANCE_PROXIMITY_DISABLE",
    67 : "DATA_GPS_PRIMARY_CHANGED",
    68 : "DATA_WINCH_RELAXED",
    69 : "DATA_WINCH_LENGTH_CONTROL",
    70 : "DATA_WINCH_RATE_CONTROL",
    71 : "DATA_ZIGZAG_STORE_A",
    72 : "DATA_ZIGZAG_STORE_B",
    73 : "DATA_LAND_REPO_ACTIVE",
    74 : "DATA_STANDBY_ENABLE",
    75 : "DATA_STANDBY_DISABLE",

    80 : "FENCE_FLOOR_ENABLE",
    81 : "FENCE_FLOOR_DISABLE",

    85 : "EK3_SOURCES_SET_TO_PRIMARY",
    86 : "EK3_SOURCES_SET_TO_SECONDARY",
    87 : "EK3_SOURCES_SET_TO_TERTIARY",

    90 : "AIRSPEED_PRIMARY_CHANGED",

    163 : "DATA_SURFACED",
    164 : "DATA_NOT_SURFACED",
    165 : "DATA_BOTTOMED",
    166 : "DATA_NOT_BOTTOMED",
}

subsystems = {
    1 : "MAIN",
    2 : "RADIO",
    3 : "COMPASS",
    4 : "OPTFLOW",
    5 : "FAILSAFE_RADIO",
    6 : "FAILSAFE_BATT",
    7 : "FAILSAFE_GPS",
    8 : "FAILSAFE_GCS",
    9 : "FAILSAFE_FENCE",
    10 : "FLIGHT_MODE",
    11 : "GPS",
    12 : "CRASH_CHECK",
    13 : "FLIP",
    14 : "AUTOTUNE",
    15 : "PARACHUTES",
    16 : "EKFCHECK",
    17 : "FAILSAFE_EKFINAV",
    18 : "BARO",
    19 : "CPU",
    20 : "FAILSAFE_ADSB",
    21 : "TERRAIN",
    22 : "NAVIGATION",
    23 : "FAILSAFE_TERRAIN",
    24 : "EKF_PRIMARY",
    25 : "THRUST_LOSS_CHECK",
    26 : "FAILSAFE_SENSORS",
    27 : "FAILSAFE_LEAK",
    28 : "PILOT_INPUT",
    29 : "FAILSAFE_VIBE",
    30 : "INTERNAL_ERROR",
    31 : "FAILSAFE_DEADRECKON",
}

error_codes = {
    "RADIO" : { # subsystem specific error codes -- radio
        2: "RADIO_LATE_FRAME",
    },
    "FAILSAFE_FENCE" : { # for failsafe fence, non-zero is a bitmask
        0: "FAILSAFE_RESOLVED",
        '*': "Fence:#",
    },
    "FAILSAFE*" : { # subsystem specific error codes -- failsafe_thr, batt, gps
        0: "FAILSAFE_RESOLVED",
        1: "FAILSAFE_OCCURRED",
    },
    "MAIN" : { # subsystem specific error codes -- main
        1: "MAIN_INS_DELAY",
    },
    "CRASH_CHECK" : { # subsystem specific error codes -- crash checker
        1: "CRASH_CHECK_CRASH",
        2: "CRASH_CHECK_LOSS_OF_CONTROL",
    },
    "FLIP" : { # subsystem specific error codes -- flip
        2: "FLIP_ABANDONED",
    },
    "TERRAIN" : { # subsystem specific error codes -- terrain
        2: "MISSING_TERRAIN_DATA",
    },
    "NAVIGATION" : { # subsystem specific error codes -- navigation
        2: "FAILED_TO_SET_DESTINATION",
        3: "RESTARTED_RTL",
        4: "FAILED_CIRCLE_INIT",
        5: "DEST_OUTSIDE_FENCE",
        6: "RTL_MISSING_RNGFND",
    },
    "INTERNAL_ERROR" : { # subsystem specific error codes -- internal_error
        1: "INTERNAL_ERRORS_DETECTED",
    },
    "PARACHUTES" : { # parachute failed to deploy because of low altitude or landed
        2: "PARACHUTE_TOO_LOW",
        3: "PARACHUTE_LANDED",
    },
    "EKFCHECK" : { # EKF check definitions
        2: "EKFCHECK_BAD_VARIANCE",
        0: "EKFCHECK_VARIANCE_CLEARED",
    },
    "BARO" : { # Baro specific error codes
        2: "BARO_GLITCH",
        3: "BAD_DEPTH", # sub-only
    },
    "GPS" : { # GPS specific error coces
        2: "GPS_GLITCH",
    },
    "EKF_PRIMARY" : { # EKF primary - code is the EKF number
        '*': "EKF:#",
    },
    "FLIGHT_MODE" : { # flight mode - code is the mode number
        '*': "Mode:#",
    },
    "*" : { # general error codes
        0: "ERROR_RESOLVED",
        1: "FAILED_TO_INITIALISE",
        4: "UNHEALTHY",
    }
}
    
def cmd_messages(args):
    '''show messages'''
    invert = False
    if len(args) > 0:
        wildcard = args[0]
        if wildcard.startswith("!"):
            invert = True
            wildcard = wildcard[1:]
        if wildcard.find('*') == -1 and wildcard.find('?') == -1:
            wildcard = "*" + wildcard + "*"
    else:
        wildcard = '*'

    # statustext reassembly:
    statustext_current_id = None
    statustext_next_seq = 0
    statustext_accumulation = None
    # statustext_timestring = None


    def print_if_match(m_timestring, mstr):
        matches = fnmatch.fnmatch(mstr.upper(), wildcard.upper())
        if invert:
            matches = not matches
        if matches:
            print("%s %s" % (m_timestring, mstr))

    def get_error_code(subsys, ecode):
        for e in error_codes:
            if e.endswith('*'):
                subsys_match = subsys.startswith(e[:-1])
            else:
                subsys_match = subsys == e
            if subsys_match:
                if ecode in error_codes[e]:
                    return error_codes[e][ecode]
                elif "*" in error_codes[e]:
                    return error_codes[e]['*'].replace("#",str(ecode))
        return str(ecode)

    mestate.mlog.rewind()
    types = set(['MSG','EV','ERR', 'STATUSTEXT'])
    while True:
        m = mestate.mlog.recv_match(type=types, condition=mestate.settings.condition)
        if m is None:
            break
        if m.get_type() == 'MSG':
            mstr = m.Message
        elif m.get_type() == 'EV':
            mstr = "Event: %s" % events.get(m.Id, str(m.Id))
        elif m.get_type() == 'ERR':
            subsys = subsystems.get(m.Subsys, str(m.Subsys))
            ecode = get_error_code(subsys, m.ECode)
            mstr = "Error: Subsys %s ECode %s " % (subsys, ecode)
        else:
            mstr = m.text

        # special handling for statustext:
        if hasattr(m, 'id') and hasattr(m, 'chunk_seq') and m.chunk_seq != 0:  # assume STATUSTEXT
            if m.id != statustext_current_id:
                if statustext_accumulation is not None:
                    print_if_match(statustext_timestring, statustext_accumulation)
                statustext_accumulation = ""
                statustext_current_id = m.id
                statustext_next_seq = 0
                statustext_timestring = timestring(m)
            if m.chunk_seq != statustext_next_seq:
                statustext_accumulation += "..."
            statustext_next_seq = m.chunk_seq + 1
            statustext_accumulation += m.text
            continue

        print_if_match(timestring(m), mstr)

    # emit any remaining statustext if it matches:
    if statustext_accumulation is not None:
        print_if_match(statustext_timestring, statustext_accumulation)

    mestate.mlog.rewind()

def extract_files():
    '''extract all FILE messages as a dictionary of files'''
    sequences = {}
    mestate.mlog.rewind()
    while True:
        m = mestate.mlog.recv_match(type=['FILE'], condition=mestate.settings.condition)
        if m is None:
            break
        if not m.FileName in sequences:
            sequences[m.FileName] = set()
        sequences[m.FileName].add((m.Offset, m.Data[:m.Length]))
    ret = {}
    for f in sequences:
        ofs = 0
        seen = set()
        seq = sorted(list(sequences[f]), key=lambda t: t[0])
        ret[f] = bytes()
        for t in seq:
            if t[0] in seen:
                continue
            seen.add(t[0])
            if t[0] != ofs:
                print("Gap in %s at %u" % (f, ofs))
            ret[f] += t[1]
            ofs = t[0]+len(t[1])

    mestate.mlog.rewind()
    return ret

def cmd_file(args):
    '''show files'''
    files = extract_files()
    if len(args) == 0:
        # list
        for n in sorted(files.keys()):
            print("%s (length %u)" % (n, len(files[n])))
        return
    fname = args[0]
    if not fname in files:
        print("File %s not found" % fname)
        return
    if len(args) == 1:
        # print on terminal
        print(files[fname].decode('utf-8'))
    else:
        # save to file
        dest = args[1]
        open(dest, "wb").write(files[fname])
        print("Saved %s to %s" % (fname, dest))

def set_vehicle_name():
    mapping = { mavutil.mavlink.MAV_TYPE_GROUND_ROVER : "Rover",
                mavutil.mavlink.MAV_TYPE_FIXED_WING : "ArduPlane",
                mavutil.mavlink.MAV_TYPE_QUADROTOR : "ArduCopter",
                mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER : "AntennaTracker",
                mavutil.mavlink.MAV_TYPE_SUBMARINE : "ArduSub",
                }
    mestate.param_help.vehicle_name = mapping.get(mestate.mlog.mav_type, None)

def cmd_param_diff(args):
    '''show parameter changed using 4.3.x defaults'''
    verbose = mestate.settings.paramdocs
    mlog = mestate.mlog
    if not hasattr(mlog, 'param_defaults'):
        print("No param defaults in log")
        return
    if len(args) > 0:
        wildcard = args[0]
    else:
        wildcard = '*'
    k = sorted(mlog.params.keys())
    defaults = mlog.param_defaults
    for p in k:
        p = str(p).upper()
        if not p in defaults:
            continue
        if mlog.params[p] == defaults[p]:
            continue
        if fnmatch.fnmatch(p, wildcard.upper()):
            s = "%-16.16s %f %f" % (str(p), mlog.params[p], defaults[p])
            if verbose:
                set_vehicle_name()
                info = mestate.param_help.param_info(p, mlog.params[p])
                if info is not None:
                    s += " # %s" % info
                info_default = mestate.param_help.param_info(p, defaults[p])
                if info_default is not None:
                    s += " (DEFAULT: %s)" % info_default
            print(s)

def cmd_param_save(args, changed_only=False):
    '''save parameters'''
    mlog = mestate.mlog
    if changed_only and not hasattr(mlog, 'param_defaults'):
        print("No param defaults in log")
        return
    filename = args[0]
    if len(args) > 1:
        wildcard = args[1]
    else:
        wildcard = '*'
    k = sorted(mlog.params.keys())
    count = 0
    try:
        f = open(filename, "w")
    except Exception as ex:
        print("Failed to open %s - %s" % (filename, ex))
        return
    for p in k:
        p = str(p).upper()
        if changed_only and p in mlog.param_defaults and mlog.params[p] == mlog.param_defaults[p]:
            continue
        if fnmatch.fnmatch(p, wildcard.upper()):
            v = mlog.params[p]
            if int(v) == v:
                s = "%-16.16s %d" % (str(p), mlog.params[p])
            else:
                s = "%-16.16s %f" % (str(p), mlog.params[p])
                s = s.rstrip('0')
            f.write(s + "\n")
            count += 1
    f.close()
    print("Saved %u parameters to %s" % (count, filename))
            
def ftp_decode(mlog):
    '''decode FILE_TRANSFER_PROTOCOL for parameters'''
    mlog.rewind()
    ftp_transfers = {}
    ftp_block_size = 0

    FTP_OpenFileRO = 4
    FTP_ReadFile = 5
    FTP_BurstReadFile = 15
    FTP_Ack = 128
    FTP_NAck = 129

    class Block(object):
        def __init__(self, offset, size, data):
            self.offset = offset
            self.size = size
            self.data = data

    class Transfer(object):
        def __init__(self, filename):
            self.filename = filename
            self.blocks = []

        def extract(self):
            self.blocks.sort(key = lambda x: x.offset)
            data = bytes()
            for b in self.blocks:
                if b.offset < len(data):
                    continue
                if b.offset > len(data):
                    print("gap at %u" % len(data))
                    return None
                data += bytes(b.data)
            return data
    
    while True:
        m = mestate.mlog.recv_match(type=['FILE_TRANSFER_PROTOCOL'])
        if m is None:
            break
        session = m.payload[2]
        opcode = m.payload[3]
        size = m.payload[4]
        req_opcode = m.payload[5]
        burst_complete = m.payload[6]
        data = m.payload[12:12+size]
        if opcode == FTP_OpenFileRO:
            filename = data
            ftp_transfers[session] = Transfer(bytearray(filename))
        if req_opcode in [FTP_ReadFile, FTP_BurstReadFile] and opcode == FTP_Ack:
            if not session in ftp_transfers:
                print("No session %u" % session)
                continue
            offset, = struct.unpack("<I", bytearray(m.payload[8:12]))
            ftp_transfers[session].blocks.append(Block(offset,size,bytearray(data)))

    pdata = None
    for session in ftp_transfers:
        f = ftp_transfers[session]
        if f.filename.decode().startswith('@PARAM/param.pck'):
            ex = f.extract()
            if ex is not None:
                pdata = param_ftp.ftp_param_decode(ex)
    if pdata is not None:
        for (name,value,ptype) in pdata.params:
            name = name.decode('utf-8')
            if name not in mlog.params:
                mlog.params[name] = value
        if pdata.defaults is not None and len(pdata.defaults) > 0:
            mlog.param_defaults = {}
            for (name,value,ptype) in pdata.defaults:
                name = name.decode('utf-8')
                mlog.param_defaults[name] = value


def cmd_param(args):
    '''show parameters'''
    verbose = mestate.settings.paramdocs
    mlog = mestate.mlog
    usage = "Usage: param <help|download|check|show|diff|save|savechanged>"
    global done_ftp_decode
    if isinstance(mlog, mavutil.mavfile) and not done_ftp_decode:
        done_ftp_decode = True
        ftp_decode(mlog)
    if len(args) > 0:
        if args[0] == 'help':
            if len(args) < 2:
                print(usage)
                return
            set_vehicle_name()
            mestate.param_help.param_help(args[1:])
            return
        if args[0] == 'download':
            mestate.param_help.param_help_download()
            return
        if args[0] == 'check':
            set_vehicle_name()
            mestate.param_help.param_check(mlog.params, args[1:])
            return
        if args[0] == 'show':
            # habits from mavproxy
            cmd_param(args[1:])
            return
        if args[0] == 'diff':
            cmd_param_diff(args[1:])
            return
        if args[0] == 'save':
            # save parameters
            if len(args) < 2:
                print("Usage: param save FILENAME <WILDCARD>")
                return
            cmd_param_save(args[1:], False)
            return
        if args[0] == 'savechanged':
            # save changed parameters
            if len(args) < 2:
                print("Usage: param savechanged FILENAME <WILDCARD>")
                return
            cmd_param_save(args[1:], True)
            return
        wildcard = args[0]
        if len(args) > 1 and args[1] == "-v":
            verbose = True
    else:
        wildcard = '*'
    k = sorted(mlog.params.keys())
    for p in k:
        if fnmatch.fnmatch(str(p).upper(), wildcard.upper()):
            s = "%-16.16s %f" % (str(p), mlog.params[p])
            if verbose:
                set_vehicle_name()
                info = mestate.param_help.param_info(p, mlog.params[p])
                if info is not None:
                    s += " # %s" % info
            print(s)

def cmd_paramchange(args):
    '''show param changes'''
    if len(args) > 0:
        wildcard = args[0]
        if wildcard.find('*') == -1 and wildcard.find('?') == -1:
            wildcard = "*" + wildcard + "*"
    else:
        wildcard = '*'
    types = set(['PARM','PARAM_VALUE'])
    vmap = {}
    while True:
        m = mestate.mlog.recv_match(type=types, condition=mestate.settings.condition)
        if m is None:
            break
        if m.get_type() == 'PARM':
            pname = m.Name
            pvalue = m.Value
        elif m.get_type() == 'PARAM_VALUE':
            pname = m.param_id
            pvalue = m.param_value
        else:
            continue
        if pname.startswith('STAT_'):
            # STAT_* changes are not interesting
            continue
        if not fnmatch.fnmatch(pname.upper(), wildcard.upper()):
            continue
        if not pname in vmap or vmap[pname] == pvalue:
            vmap[pname] = pvalue
            continue

        print("%s %s %.6f -> %.6f" % (timestring(m), pname, vmap[pname], pvalue))
        vmap[pname] = pvalue
    mestate.mlog.rewind()


def cmd_logmessage(args):
    '''show log message information'''
    mlog = mestate.mlog
    usage = "Usage: logmessage <help|download>"
    # Print usage and return, if we have no arguments
    if len(args) <= 0:
        print(usage)
        return
    # help: print help for the requested log message
    if args[0] == 'help':
        if len(args) < 2:
            print(usage)
            return
        if hasattr(mlog, 'metadata'):
            mlog.metadata.print_help(args[1])
        elif isinstance(mlog, mavutil.mavlogfile):
            print("logmessage help is not supported for telemetry log files")
        else:
            print("Incompatible pymavlink; upgrade pymavlink?")
        return
    # download: download XML files for log messages
    if args[0] == 'download':
        if not hasattr(DFReader, 'DFMetaData'):
            print("Incompatible pymavlink; upgrade pymavlink?")
            return
        try:
            child = multiproc.Process(target=DFReader.DFMetaData.download)
            child.start()
        except Exception as e:
            print(e)
        if hasattr(mlog, 'metadata'):
            mlog.metadata.reset()
        return
    # Print usage if we've dropped through the ifs
    print(usage)


def cmd_mission(args):
    '''show mission'''
    if (len(args) == 1):
        print("Usage: mission <save FILENAME>")
        return
    mestate.mlog.rewind()
    types = set(['CMD','MISSION_ITEM_INT'])
    wp = mavwp.MAVWPLoader()
    while True:
        m = mestate.mlog.recv_match(type=types, condition=mestate.settings.condition)
        if m is None:
            break
        if m.get_type() == 'CMD':
            try:
                frame = m.Frame
            except AttributeError:
                print("Warning: assuming frame is GLOBAL_RELATIVE_ALT")
                frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            num_wps = m.CTot
            m = mavutil.mavlink.MAVLink_mission_item_message(0,
                                                             0,
                                                             m.CNum,
                                                             frame,
                                                             m.CId,
                                                             0, 1,
                                                             m.Prm1, m.Prm2, m.Prm3, m.Prm4,
                                                             m.Lat, m.Lng, m.Alt)

        if m.get_type() == 'MISSION_ITEM_INT':
            m = mavutil.mavlink.MAVLink_mission_item_message(m.target_system,
                                                             m.target_component,
                                                             m.seq,
                                                             m.frame,
                                                             m.command,
                                                             m.current,
                                                             m.autocontinue,
                                                             m.param1,
                                                             m.param2,
                                                             m.param3,
                                                             m.param4,
                                                             m.x*1.0e-7,
                                                             m.y*1.0e-7,
                                                             m.z)
        if m.current >= 2:
            continue

        while m.seq > wp.count():
            print("Adding dummy WP %u" % wp.count())
            wp.set(m, wp.count())
        wp.set(m, m.seq)
    if len(args) == 2 and args[0] == 'save':
        wp.save(args[1])
        mestate.mlog.rewind()
        return
    for i in range(wp.count()):
        w = wp.wp(i)
        print("%u\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t\t%f\t\t%f\t%u" % (
            w.seq, w.current, w.frame, w.command,
            w.param1, w.param2, w.param3, w.param4,
            w.x, w.y, w.z, w.autocontinue))
    mestate.mlog.rewind()
    
def cmd_devid(args):
    '''show parameters'''
    params = mestate.mlog.params
    k = sorted(params.keys())
    for p in k:
        if p.startswith('COMPASS_DEV_ID') or p.startswith('COMPASS_PRIO') or (
                p.startswith('COMPASS') and p.endswith('DEV_ID')):
            mp_util.decode_devid(params[p], p)
        if p.startswith('INS') and p.endswith('_ID'):
            mp_util.decode_devid(params[p], p)
        if p.startswith('GND_BARO') and p.endswith('_ID'):
            mp_util.decode_devid(params[p], p)
        if p.startswith('BARO') and p.endswith('DEVID'):
            mp_util.decode_devid(params[p], p)
        if p.startswith('ARSPD') and p.endswith('DEVID'):
            mp_util.decode_devid(params[p], p)

def cmd_loadfile(args):
    '''callback from menu to load a log file'''
    if len(args) != 1:
        fileargs = " ".join(args)
    else:
        fileargs = args[0]
    if not os.path.exists(fileargs):
        print("Error loading file ", fileargs);
        return
    if os.name == 'nt':
        #convert slashes in Windows
        fileargs = fileargs.replace("\\", "/")
    loadfile(fileargs.strip('"'))

def loadfile(args):
    '''load a log file (path given by arg)'''
    mestate.console.write("Loading %s...\n" % args)
    t0 = time.time()
    mlog = mavutil.mavlink_connection(args, notimestamps=False,
                                      zero_time_base=False,
                                      progress_callback=progress_bar)
    mestate.filename = args
    mestate.mlog = mlog
    # note that this is a shallow copy of the messages.
    # Instance-number-containing messages in mestate.status.msgs may
    # reference messages in their parent DFReader object which no
    # longer exist after a rewind() is performed.  Still, this is good
    # enough for tab-completion to function.
    mestate.status.msgs = copy.copy(mlog.messages)
    t1 = time.time()
    mestate.console.write("\ndone (%u messages in %.1fs)\n" % (mestate.mlog._count, t1-t0))

    # evaluate graph expressions before finding flightmode list as
    # flightmode_list does a rewind(), and that clears the DFReader
    # object.  While we do take a copy of mestate.status.msgs above,
    # that is a shallow copy, so does not allow
    # DFMessage.parent.messages to function, which is vital for
    # instance-number-indexing to work - and the graph expression
    # evaluation requires that to function.
    load_graphs()

    global flightmodes
    flightmodes = mlog.flightmode_list()

    mestate.mav_param = mlog.params

    setup_menus()

def print_caught_exception(e):
    if sys.version_info[0] >= 3:
        ret = "%s\n" % e
        ret += ''.join(traceback.format_exception(type(e),
                                                  value=e,
                                                  tb=e.__traceback__))
        print(ret)
    else:
        print(traceback.format_exc(e))

def cmd_help(args):
    '''help command'''
    if len(args) == 0:
        k = command_map.keys()
        for cmd in sorted(k):
            (fn, help) = command_map[cmd]
            print("%-15s : %s" % (cmd, help))
        return
    cmd = args[0]
    if cmd in command_map.keys():
        (fn, help) = command_map[cmd]
        print("%-15s : %s" % (cmd, help))
        return
    import pymavlink.mavextra as mavextra
    import math
    import pydoc
    pydoc.pager = pydoc.plainpager
    for v in [mavextra,math]:
        if hasattr(v,cmd):
            pydoc.help(getattr(v,cmd))
            return
    print("%s not found" % cmd)

def process_stdin(line):
    '''handle commands from user'''
    if line is None:
        sys.exit(0)

    line = line.strip()
    if not line:
        return

    try:
        args = shlex.split(line)
    except ValueError as e:
        print_caught_exception(e)
        return

    cmd = args[0]
    if cmd == 'help':
        cmd_help(args[1:])
        return
    if cmd == 'exit':
        mestate.exit = True
        return

    if not cmd in command_map:
        print("Unknown command '%s'" % line)
        return
    (fn, help) = command_map[cmd]
    try:
        fn(args[1:])
    except Exception as e:
        print("ERROR in command %s: %s" % (args[1:], str(e)))
        if mestate.settings.debug > 0:
            print_caught_exception(e)

def input_loop():
    '''wait for user input'''
    while mestate.exit != True:
        try:
            if mestate.exit != True:
                line = input(mestate.rl.prompt)
        except EOFError:
            mestate.exit = True
            sys.exit(1)
        mestate.input_queue.put(line)

new_matplotlib = False
mversion = matplotlib.__version__.split('.')
matplotlib_major = mversion[0]
matplotlib_minor = mversion[1]
if int(matplotlib_major) > 3:
    new_matplotlib = True
elif (int(matplotlib_major) >= 3 and int(matplotlib_minor) >= 5):
    new_matplotlib = True

def epoch_time(num):
    '''converts a number into an epoch time - compatability wrapper'''
    if new_matplotlib:
        return matplotlib.dates.num2date(num).timestamp()
    return matplotlib.dates.num2epoch(num)

def main_loop():
    '''main processing loop, display graphs and maps'''
    global grui, xlimits
    while True:
        if mestate is None or mestate.exit:
            return
        while not mestate.input_queue.empty():
            line = mestate.input_queue.get()
            cmds = line.split(';')
            for c in cmds:
                process_stdin(c)

        remlist = []
        for i in range(0, len(grui)):
            xlim = grui[i].check_xlim_change()
            if xlim is not None and mestate.settings.sync_xzoom:
                for j in range(0, len(grui)):
                    if j == i:
                        continue
                    if not grui[j].set_xlim(xlim):
                        remlist.append(j)
                xlimits.last_xlim = xlim
                from dateutil.tz import tzlocal
                localtimezone = tzlocal()
                tbase = epoch_time(xlim[0])
                tzofs = localtimezone.utcoffset(datetime.datetime.utcfromtimestamp(tbase)).total_seconds()
                xlimits.xlim_low = epoch_time(xlim[0]) - tzofs
                xlimits.xlim_high = epoch_time(xlim[1]) - tzofs
                
                if mestate.settings.sync_xmap:
                    remlist = []
                    global map_timelim_pipes
                    for p in map_timelim_pipes[:]:
                        try:
                            p[0].send(xlim)
                        except Exception:
                            map_timelim_pipes.remove(p)
                break
        if len(remlist) > 0:
            # remove stale graphs
            new_grui = []
            for j in range(0, len(grui)):
                if j not in remlist:
                    new_grui.append(grui[j])
            grui = new_grui

        time.sleep(0.1)


command_map = {
    'graph'      : (cmd_graph,     'display a graph'),
    'set'        : (cmd_set,       'control settings'),
    'reload'     : (cmd_reload,    'reload graphs'),
    'save'       : (cmd_save,      'save a graph'),
    'condition'  : (cmd_condition, 'set graph conditions'),
    'param'      : (cmd_param,     'show parameters'),
    'paramchange': (cmd_paramchange, 'show parameter changes in log'),
    'messages'   : (cmd_messages,  'show messages'),
    'devid'      : (cmd_devid,     'show device IDs'),
    'map'        : (cmd_map,       'show map view'),
    'fft'        : (cmd_fft,       'show a FFT (if available)'),
    'loadLog'    : (cmd_loadfile,  'load a log file'),
    'stats'      : (cmd_stats,     'show statistics on the log'),
    'magfit'     : (cmd_magfit,    'fit mag parameters to WMM'),
    'dump'       : (cmd_dump,      'dump messages from log'),
    'file'       : (cmd_file,      'show files'),
    'mission'    : (cmd_mission,   'show mission'),
    'logmessage' : (cmd_logmessage, 'show log message information'),
    }

def progress_bar(pct):
    if pct % 2 == 0:
        mestate.console.write('#')

if __name__ == "__main__":
    multiproc.freeze_support()
    from argparse import ArgumentParser
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--version", action='store_true', help="show version")
    parser.add_argument("files", metavar="<FILE>", nargs="?")
    args = parser.parse_args()

    if args.version:
        #pkg_resources doesn't work in the windows exe build, so read the version file
        try:
            version = pkg_resources.require("mavproxy")[0].version
        except Exception as e:
            start_script = mp_util.dot_mavproxy("version.txt")
            f = open(start_script, 'r')
            version = f.readline()
        print("MAVExplorer Version: " + version)
        sys.exit(1)
    
    mestate = MEState()
    setup_file_menu()

    mestate.rl = rline.rline("MAV> ", mestate)

    #If specified, open the log file
    if args.files is not None and len(args.files) != 0:
        loadfile(args.files)

    # run main loop as a thread
    mestate.thread = threading.Thread(target=main_loop, name='main_loop')
    mestate.thread.daemon = True
    mestate.thread.start()

    # input loop
    while mestate.rl is not None and not mestate.exit:
        try:
            try:
                line = mestate.rl.input()
            except EOFError:
                mestate.exit = True
                break
            mestate.input_queue.put(line)
        except KeyboardInterrupt:
            mestate.exit = True
            break

