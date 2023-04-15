defmodule FleetMonitor.Impl.ConfigReader do
  @path Path.join(:code.priv_dir(:fleet_monitor), "current_level.yaml")
  @external_resource @path
  @config elem(YamlElixir.read_from_file(@path), 1)

  def get_conf, do: @config
  def get_locations, do: @config["L1"]["locations"]
  def get_nav_graph, do: @config["L1"]["nav_graph"]
  def get_map_info, do: @config["L1"]["map"]
  def get_image_topics, do: @config["image_topics"]
end
