defmodule FleetMonitor do
  alias FleetMonitor.Runtime.Server

  defdelegate get_image, to: Server
  defdelegate subscribe(pid), to: Server
end
