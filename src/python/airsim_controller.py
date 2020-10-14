import airsim

client = airsim.MultirotorClient()

png_image = client.simGetImage("0", airsim.ImageType.Scene)

