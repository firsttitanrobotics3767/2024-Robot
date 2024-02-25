# Settings:
team = 3767
debug = True
device_name = "Orange Pi"
loopTime = 0

def dbg(text: str, color: str = "w") -> None:
    """
    This function is used for printing debug messages to the console.

    Args:
        text (str): The text to be printed.
        color (str, optional): The color of the text. Defaults to "w".

    Available colors:
        "r": red
        "w": white (default)

    """
    if debug:
        if color == "r": #red
            print(f"\033[0;31;40m{text}\033[0;37;40m")
        elif color == "w": #white
            print(f"\033[0;37;40m{text}\033[0;37;40m")
        else: #default - white
            print(f"\033[0;37;40m{text}\033[0;37;40m")

item = "imports"
dbg(f"Attempting process '{item}'...")
while True:
    try:
        import board
        import adafruit_vl53l4cd
        import adafruit_tca9548a
        from ntcore import NetworkTableInstance
        from random import randint
        import time

        dbg(f"Executed process '{item}'")
        break
    except Exception as e:
        dbg(f"Process '{item}' failed with exception:", "r")
        dbg(e, "r")
        dbg(f"Reattempting process '{item}'...")
        pass

def bus_init() -> bool:
    """
    Initialize the I2C bus.

    Returns:
        True if the I2C bus was initialized successfully, False otherwise.
    """
    item = "I2C bus init"
    dbg(f"Attempting process '{item}'...")
    try:
        global i2c
        i2c = board.I2C()

        dbg(f"Executed process '{item}'")
        return True
    except Exception as e:
        dbg(f"Process '{item}' failed with exception:", "r")
        dbg(e, "r")
        return False

def mult_conn() -> bool:
    """
    Initialize the TCA9548A I2C multiplexer.

    Returns:
        True if the multiplexer was initialized successfully, False otherwise.
    """
    item = "Multiplexer connection"
    dbg(f"Attempting process '{item}'...")
    try:
        global tca
        tca = adafruit_tca9548a.TCA9548A(i2c)

        dbg(f"Executed process '{item}'")
        return True
    except Exception as e:
        dbg(f"Process '{item}' failed with exception:", "r")
        dbg(e, "r")
        return False

def intake_init() -> bool:
    """
    Initialize the intake sensor.

    Returns:
        True if the intake sensor was initialized successfully, False otherwise.
    """
    item = "Intake sensor init"
    dbg(f"Attempting process '{item}'...")
    try:
        global intake
        intake = adafruit_vl53l4cd.VL53L4CD(tca[1])
        intake.inter_measurement = 0
        intake.timing_budget = 10
        intake.start_ranging()

        dbg(f"Executed process '{item}'")
        return True
    except Exception as e:
        dbg(f"Process '{item}' failed with exception:", "r")
        dbg(e, "r")
        return False

def intake_init() -> bool:
    """
    Initialize the intake sensor.

    Returns:
        True if the intake sensor was initialized successfully, False otherwise.
    """
    item = "Intake sensor init"
    dbg(f"Attempting process '{item}'...")
    try:
        global intake
        intake = adafruit_vl53l4cd.VL53L4CD(tca[1])
        intake.inter_measurement = 0
        intake.timing_budget = 10
        intake.start_ranging()

        dbg(f"Executed process '{item}'")
        return True
    except Exception as e:
        dbg(f"Process '{item}' failed with exception:", "r")
        dbg(e, "r")
        return False

def shooter_init() -> bool:
    """
    Initialize the shooter sensor.

    Returns:
        True if the shooter sensor was initialized successfully, False otherwise.
    """
    item = "Shooter sensor init"
    dbg(f"Attempting process '{item}'...")
    try:
        global shooter
        shooter = adafruit_vl53l4cd.VL53L4CD(tca[2])
        shooter.inter_measurement = 0
        shooter.timing_budget = 10
        shooter.start_ranging()

        dbg(f"Executed process '{item}'")
        return True
    except Exception as e:
        dbg(f"Process '{item}' failed with exception:", "r")
        dbg(e, "r")
        return False

def setup_nt() -> bool:
    """
    Initialize the NetworkTables client.

    Returns:
        True if the NetworkTables client was initialized successfully, False otherwise.
    """
    item = "Setup network tables"
    dbg(f"Attempting process '{item}'...")
    try:
        global table
        inst.startClient4(device_name)
        inst.setServerTeam(team)
        inst.startDSClient()
        table = inst.getTable("SmartDashboard")

        dbg(f"Executed process '{item}'")
        return True
    except Exception as e:
        dbg(f"Process '{item}' failed with exception:", "r")
        dbg(e, "r")
        return False

def shooter_pub_init() -> bool:
    """
    Initialize the shooter data publishing.

    Returns:
        True if the shooter data publishing was initialized successfully, False otherwise.
    """
    item = "Shooter pub init"
    dbg(f"Attempting process '{item}'...")
    try:
        global shooter_pub
        shooter_pub = table.getDoubleTopic("shooter").publish()

        dbg(f"Executed process '{item}'")
        return True
    except Exception as e:
        dbg(f"Process '{item}' failed with exception:", "r")
        dbg(e, "r")
        return False

def intake_pub_init() -> bool:
    """
    Initialize the intake data publishing.

    Returns:
        True if the intake data publishing was initialized successfully, False otherwise.
    """
    item = "Intake pub init"
    dbg(f"Attempting process '{item}'...")
    try:
        global intake_pub
        intake_pub = table.getDoubleTopic("intake").publish()

        dbg(f"Executed process '{item}'")
        return True
    except Exception as e:
        dbg(f"Process '{item}' failed with exception:", "r")
        dbg(e, "r")
        return False

def shooter_clear() -> bool:
    """
    Clears the interrupt flag for the shooter sensor.

    Returns:
        True if the interrupt flag was cleared successfully, False otherwise.
    """
    item = "Shooter interrupt clear"
    dbg(f"Attempting process '{item}'...")
    try:
        shooter.clear_interrupt()

        dbg(f"Executed process '{item}'")
        return True
    except Exception as e:
        dbg(f"Process '{item}' failed with exception:", "r")
        dbg(e, "r")
        return False
        
def intake_clear() -> bool:
    """
    Clears the interrupt flag for the intake sensor.

    Returns:
        True if the interrupt flag was cleared successfully, False otherwise.
    """
    item = "Intake interrupt clear"
    dbg(f"Attempting process '{item}'...")
    try:
        intake.clear_interrupt()

        dbg(f"Executed process '{item}'")
        return True
    except Exception as e:
        dbg(f"Process '{item}' failed with exception:", "r")
        dbg(e, "r")
        return False

def shooter_set() -> bool:
    """
    This function is used to publish the distance data from the shooter sensor to the NetworkTables.

    Returns:
        True if the distance data was published successfully, False otherwise.
    """
    item = "Publish shooter data"
    dbg(f"Attempting process '{item}'...")
    try:
        shooter_pub.set(shooter.distance)
        dbg(shooter.distance)

        dbg(f"Executed process '{item}'")
        return True
    except Exception as e:
        dbg(f"Process '{item}' failed with exception:", "r")
        dbg(e, "r")
        return False

def intake_set() -> bool:
    """
    This function is used to publish the distance data from the intake sensor to the NetworkTables.

    Returns:
        True if the distance data was published successfully, False otherwise.
    """
    item = "Publish intake data"
    dbg(f"Attempting process '{item}'...")
    try:
        intake_pub.set(intake.distance)
        dbg(intake.distance)

        dbg(f"Executed process '{item}'")
        return True
    except Exception as e:
        dbg(f"Process '{item}' failed with exception:", "r")
        dbg(e, "r")
        return False
    
def intake_alive_init() -> bool:
    """
    Initialize the intake alive topic.

    Returns:
        True if the intake alive topic was initialized successfully, False otherwise.
    """
    item = "Intake alive init"
    dbg(f"Attempting process '{item}'...")
    try:
        global intake_alive_pub
        intake_alive_pub = table.getBooleanTopic("intake_alive").publish()

        dbg(f"Executed process '{item}'")
        return True
    except Exception as e:
        dbg(f"Process '{item}' failed with exception:", "r")
        dbg(e, "r")
        return False

def shooter_alive_init() -> bool:
    """
    Initialize the shooter alive topic.

    Returns:
        True if the shooter alive topic was initialized successfully, False otherwise.
    """
    item = "Shooter alive init"
    dbg(f"Attempting process '{item}'...")
    try:
        global shooter_alive_pub
        shooter_alive_pub = table.getBooleanTopic("shooter_alive").publish()

        dbg(f"Executed process '{item}'")
        return True
    except Exception as e:
        dbg(f"Process '{item}' failed with exception:", "r")
        dbg(e, "r")
        return False

num = 0
def heartbeat() -> bool:
    """
    This function is used for publishing a heartbeat to the network tables.

    The heartbeat is a counter that increments by 1 every loop. The counter is reset to 0 after reaching the maximum value of 3,000,000.

    Returns:
        True if the heartbeat was published successfully, False otherwise.
    """
    item = "Heartbeat"
    dbg(f"Attempting process '{item}'...")
    try:
        global num
        num += 1
        if num >= 3000000:
            num = 0
        heartbeat_pub.set(num)

        dbg(f"Executed process '{item}'")
        return True
    except Exception as e:
        dbg(f"Process '{item}' failed with exception:", "r")
        dbg(e, "r")
        return False

def intake_alive_set(isAlive: bool) -> bool:
    """
    This function is used to publish the intake alive status to the NetworkTables.

    Args:
        isAlive (bool): A boolean value indicating whether the intake is alive or not.

    Returns:
        bool: A boolean value indicating whether the function succeeded or not.
    """
    item = "Publish intake alive data"
    dbg(f"Attempting process '{item}'...")
    try:
        intake_alive_pub.set(isAlive)

        dbg(f"Executed process '{item}'")
        return True
    except Exception as e:
        dbg(f"Process '{item}' failed with exception:", "r")
        dbg(e, "r")
        return False

def shooter_alive_set(isAlive: bool) -> bool:
    """
    This function is used to publish the shooter alive status to the NetworkTables.

    Args:
        isAlive (bool): A boolean value indicating whether the shooter is alive or not.

    Returns:
        bool: A boolean value indicating whether the function succeeded or not.
    """
    item = "Publish shooter alive data"
    dbg(f"Attempting process '{item}'...")
    try:
        shooter_alive_pub.set(isAlive)

        dbg(f"Executed process '{item}'")
        return True
    except Exception as e:
        dbg(f"Process '{item}' failed with exception:", "r")
        dbg(e, "r")
        return False

def heartbeat_init() -> bool:
    """
    Initialize the heartbeat publishing.

    Returns:
        True if the heartbeat publishing was initialized successfully, False otherwise.
    """
    item = "Heartbeat init"
    dbg(f"Attempting process '{item}'...")
    try:
        global heartbeat_pub
        heartbeat_pub = table.getDoubleTopic("orangepi_heartbeat").publish()

        dbg(f"Executed process '{item}'")
        return True
    except Exception as e:
        dbg(f"Process '{item}' failed with exception:", "r")
        dbg(e, "r")
        return False

def intake_alive_smart(isAlive: bool) -> None:
    """
    This function is used to handle the intake alive status.

    Args:
        isAlive (bool): A boolean value indicating whether the intake is alive or not.
    """
    if intake_alive_set(isAlive):
        pass
    else:
        global intake_alive_init_needed
        intake_alive_init_needed = True

def shooter_alive_smart(isAlive: bool) -> None:
    """
    This function is used to handle the shooter alive status.

    Args:
        isAlive (bool): A boolean value indicating whether the shooter is alive or not.
    """
    if shooter_alive_set(isAlive):
        pass
    else:
        global shooter_alive_init_needed
        shooter_alive_init_needed = True

def heartbeat_smart() -> None:
    """
    This function is used for publishing a heartbeat to the network tables.

    The heartbeat is a counter that increments by 1 every loop. The counter is reset to 0 after reaching the maximum value of 3,000,000.

    Returns:
        True if the heartbeat was published successfully, False otherwise.
    """
    item = "Heartbeat"
    dbg(f"Attempting process '{item}'...")
    try:
        global num
        num += 1
        if num >= 3000000:
            num = 0
        heartbeat_pub.set(num)

        dbg(f"Executed process '{item}'")
        return True
    except Exception as e:
        dbg(f"Process '{item}' failed with exception:", "r")
        dbg(e, "r")
        return False

get_nt_needed = True
setup_nt_needed = False

intake_pub_init_needed = False
shooter_pub_init_needed = False
intake_alive_init_needed = False
shooter_alive_init_needed = False
heartbeat_init_needed = False

bus_init_needed = False
mult_conn_needed = False
intake_init_needed = False
shooter_init_needed = False

intake_set_able = False
intake_clear_needed = False
shooter_set_able = False
shooter_clear_needed = False


while True:
    time.sleep(loopTime)
    dbg("-----------------------------------------------")
    if get_nt_needed:
        if get_nt():
            get_nt_needed = False
            setup_nt_needed = True
        else:
            get_nt_needed = True
    
    if setup_nt_needed:
        if setup_nt():
            setup_nt_needed = False
            intake_pub_init_needed = True
            shooter_pub_init_needed = True
            intake_alive_init_needed = True
            shooter_alive_init_needed = True
            heartbeat_init_needed = True
        else:
            setup_nt_needed = False
            get_nt_needed = True

    if intake_pub_init_needed:
        if intake_pub_init():
            intake_pub_init_needed = False
            bus_init_needed = True
        else:
            intake_pub_init_needed = False
            setup_nt_needed = True

    if shooter_pub_init_needed:
        if shooter_pub_init():
            shooter_pub_init_needed = False
            bus_init_needed = True
        else:
            shooter_pub_init_needed = False
            setup_nt_needed = True

    if intake_alive_init_needed:
        if intake_alive_init():
            intake_alive_init_needed = False
        else:
            intake_alive_init_needed = False
            setup_nt_needed = True

    if shooter_alive_init_needed:
        if shooter_alive_init():
            shooter_alive_init_needed = False
        else:
            shooter_alive_init_needed = False
            setup_nt_needed = True

    if heartbeat_init_needed:
        if heartbeat_init():
            heartbeat_init_needed = False
        else:
            heartbeat_init_needed = False
            setup_nt_needed = True

    if bus_init_needed:
        if bus_init():
            bus_init_needed = False
            mult_conn_needed = True
        else:
            bus_init_needed = True

    if mult_conn_needed:
        if mult_conn():
            mult_conn_needed = False
            intake_init_needed = True
            shooter_init_needed = True
        else:
            mult_conn_needed = False
            bus_init_needed = True

    if intake_init_needed:
        if intake_init():
            intake_init_needed = False
            intake_set_able = True
        else:
            intake_init_needed = False
            mult_conn_needed = True
    
    if shooter_init_needed:
        if shooter_init():
            shooter_init_needed = False
            shooter_set_able = True
        else:
            shooter_init_needed = False
            mult_conn_needed = True

    if intake_set_able:
        if intake_set():
            intake_clear_needed = True
            intake_alive_smart(True)
        else:
            intake_set_able = False
            intake_init_needed = True
            intake_pub_init_needed = True
            intake_alive_smart(False)

    if shooter_set_able:
        if shooter_set():
            shooter_clear_needed = True
            shooter_alive_smart(True)
        else:
            shooter_set_able = False
            shooter_init_needed = True
            shooter_pub_init_needed = True
            shooter_alive_smart(False)

    if intake_clear_needed:
        if intake_clear():
            intake_clear_needed = False
        else:
            intake_init_needed = True

    if shooter_clear_needed:
        if shooter_clear():
            shooter_clear_needed = False
        else:
            shooter_init_needed = False

    heartbeat_smart()