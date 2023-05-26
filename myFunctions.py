import vlc

def playAudio(file_path):
        instance = vlc.Instance('--no-xlib')
        media = instance.media_new(file_path)
        player = instance.media_player_new()
        player.set_media(media)
        player.play()
        while player.get_state() != vlc.State.Ended:
            pass
        player.stop()
        media.release()
        player.release()
        instance.release()

def read_raw_data(bus, addressI2C, addressRegistre):
    high = bus.read_byte_data(addressI2C, addressRegistre)
    low = bus.read_byte_data(addressI2C, addressRegistre+1)
    # *Les valeurs retournées sont probablement écrites sur plus d'un registre.*
    value = ((high << 8) | low)
    # *Oui, car on fait un décalage de ce qu'on lit dans le registre high vers le haut.*
    if(value > 32768):
        value = value - 65536
    # *C'est probablement un recalibrage.*
    return value