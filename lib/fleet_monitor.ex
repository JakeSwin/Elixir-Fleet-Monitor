defmodule FleetMonitor do
  alias FleetMonitor.Runtime.Server

  defdelegate get_image, to: Server
end
