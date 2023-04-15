defmodule FleetMonitor do
  alias FleetMonitor.Runtime.CameraReader
  alias FleetMonitor.Runtime.TaskSubmitter
  alias FleetMonitor.Impl.ConfigReader

  defdelegate get_image(topic), to: CameraReader
  defdelegate submit_task(start_name, finish_name), to: TaskSubmitter
  defdelegate get_conf, to: ConfigReader
  defdelegate get_locations, to: ConfigReader
  defdelegate get_nav_graph, to: ConfigReader
  defdelegate get_map_info, to: ConfigReader
  defdelegate get_image_topics, to: ConfigReader
end
