using AR.Drone.Control.Command;
using AR.Drone.Configuration.Section;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using AR.Drone.Control;

namespace AR.Drone.Configuration.Section
{
	class VideoConfigSection : AbstractConfigSection
	{

		public VideoConfigSection(ref CommandSender commandSender)
			:base(ConfigSection.VIDEO, ref commandSender) { }

		internal int camif_fps 
 		{
			get { return GetIntValue("camif_fps"); }
 		} 

 		internal int camif_buffers 
 		{
			get { return GetIntValue("camif_buffers"); }
 		} 

 		internal int num_trackers 
 		{
			get { return GetIntValue("num_trackers"); }
 		} 

 		internal int video_storage_space 
 		{
			get { return GetIntValue("video_storage_space"); }
 		} 

 		internal string video_on_usb 
 		{
			get { return GetStringValue("video_on_usb"); }
			set { Set("video_on_usb", value); }
 		} 

 		internal int video_file_index 
 		{
			get { return GetIntValue("video_file_index"); }
			set { Set("video_file_index", value); }
 		} 

 		internal int bitrate 
 		{
			get { return GetIntValue("bitrate"); }
			set { Set("bitrate", value); }
 		} 

 		internal int bitrate_ctrl_mode 
 		{
			get { return GetIntValue("bitrate_ctrl_mode"); }
			set { Set("bitrate_ctrl_mode", value); }
 		} 

 		internal int bitrate_storage 
 		{
			get { return GetIntValue("bitrate_storage"); }
			set { Set("bitrate_storage", value); }
 		} 

 		internal int codec_fps 
 		{
			get { return GetIntValue("codec_fps"); }
			set { Set("codec_fps", value); }
 		} 

 		internal int video_codec 
 		{
			get { return GetIntValue("video_codec"); }
			set { Set("video_codec", value); }
 		} 

 		internal int video_slices 
 		{
			get { return GetIntValue("video_slices"); }
			set { Set("video_slices", value); }
 		} 

 		internal int video_live_socket 
 		{
			get { return GetIntValue("video_live_socket"); }
			set { Set("video_live_socket", value); }
 		} 

 		internal int max_bitrate 
 		{
			get { return GetIntValue("max_bitrate"); }
			set { Set("max_bitrate", value); }
 		} 

 		internal int video_channel 
 		{
			get { return GetIntValue("video_channel"); }
			set { Set("video_channel", value); }
 		} 

 		internal float exposure_mode 
 		{
			get { return GetFloatValue("exposure_mode"); }
			set { Set("exposure_mode", value); }
 		} 

 		internal int saturation_mode 
 		{
			get { return GetIntValue("saturation_mode"); }
			set { Set("saturation_mode", value); }
 		} 

 		internal float whitebalance_mode 
 		{
			get { return GetFloatValue("whitebalance_mode"); }
			set { Set("whitebalance_mode", value); }
 		} 

 }}